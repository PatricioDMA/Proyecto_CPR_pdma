import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import math
import hashlib  # Para generar un hash único de la trayectoria

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Inicialización de variables temporales
        self.start_time = None
        self.end_time = None

        # Activación el control de Pure Pursuit
        self.active = False
        self.last_trajectory_hash = None  # Para evitar trayectorias repetidas

        self.lookahead_distance = 1.5  # metros
        self.current_pose = None
        self.trajectory = []
        self.trajectory_completed = False

        # OPCIÓN 1A: Suscribirse a la odometría sin ruido
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # OPCIÓN 1B: Suscribirse a la odometría con ruido gaussiano
        # self.create_subscription(Odometry, '/ego_racecar/odom_noisy', self.odom_callback, 10)

        # Suscribirse a la trayectoria planificada
        self.create_subscription(Path, '/planned_trajectory', self.path_callback, 10)

        # Suscribirse al LIDAR para detectar obstáculos
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.min_obstacle_distance = 0.8  # metros
        self.lidar_front_angles = (-30, 30)  # en grados
        self.obstacle_detected = False

        # OPCIÓN 2A: Publicar directamente al controlador (sin errores)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # OPCIÓN 2B: Publicar al nodo inyectador de errores sistemáticos (steering bias)
        # self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive_raw', 10)

        # Publicar el punto y la línea de avance para visualización
        self.lookahead_pub = self.create_publisher(PoseStamped, '/lookahead_point', 10)
        self.line_pub = self.create_publisher(Marker, '/lookahead_line', 10)

        # Modo normal: 
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz

        # Modo con latencia:
        # self.timer = self.create_timer(0.2, self.control_loop) # 5 Hz

        # Para evasión de obstáculos
        self.free_left = False
        self.free_right = False
        self.obstacle_marker_pub = self.create_publisher(Marker, '/obstacle_marker', 10)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.odom_header = msg.header

    def path_callback(self, msg):
        # Generar un hash simple para evitar trayectorias repetidas
        traj_bytes = str([(p.pose.position.x, p.pose.position.y) for p in msg.poses]).encode()
        current_hash = hashlib.md5(traj_bytes).hexdigest()
        
        if current_hash == self.last_trajectory_hash:
            self.get_logger().info("Trayectoria repetida ignorada.")
            return
        
        # Guardar el hash de la trayectoria actual
        self.last_trajectory_hash = current_hash
        self.trajectory = msg.poses
        self.trajectory_completed = False
        self.active = True  # activamos el control de nuevo
        self.get_logger().info(f"Nueva trayectoria recibida con {len(self.trajectory)} puntos.")
    
    def lidar_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        # Convertir ángulos a índices
        def angle_to_index(angle_deg):
            angle_rad = math.radians(angle_deg)
            return int((angle_rad - angle_min) / angle_increment)

        # Inicializar variables de estado
        self.obstacle_detected = False
        self.free_left = False
        self.free_right = False

        # Rango frontal para detección de obstáculos
        start_i = angle_to_index(self.lidar_front_angles[0])
        end_i = angle_to_index(self.lidar_front_angles[1])
        start_i = max(0, start_i)
        end_i = min(len(ranges) - 1, end_i)

        # Buscar obstáculo frontal
        for i in range(start_i, end_i + 1):
            d = ranges[i]
            if msg.range_min < d < self.min_obstacle_distance:
                self.obstacle_detected = True

        # Buscar espacio libre a izquierda y derecha
        left_range = ranges[angle_to_index(60):angle_to_index(90)]
        right_range = ranges[angle_to_index(-90):angle_to_index(-60)]

        self.free_left = any(d > self.min_obstacle_distance + 0.5 for d in left_range)
        self.free_right = any(d > self.min_obstacle_distance + 0.5 for d in right_range)

        # Visualizar el obstáculo frontal más cercano
        min_distance = float('inf')
        obstacle_angle = 0.0
        for i in range(start_i, end_i + 1):
            d = ranges[i]
            if msg.range_min < d < min_distance:
                min_distance = d
                obstacle_angle = angle_min + i * angle_increment

        if self.obstacle_detected:
            x = min_distance * math.cos(obstacle_angle)
            y = min_distance * math.sin(obstacle_angle)

            # Publicar marcador del obstáculo detectado
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacle'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 0

            self.obstacle_marker_pub.publish(marker)

    def control_loop(self):
        if not self.current_pose or not self.trajectory:
            return
        
        if not self.active:
            return
        
        # Inicializar el tiempo de inicio
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        # Evasión reactiva de obstáculos
        if self.obstacle_detected:
            self.get_logger().warn("Obstáculo detectado. Buscando desvío...")

            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = self.get_clock().now().to_msg()
            drive_msg.header.frame_id = 'base_link'
            drive_msg.drive.speed = 0.5

            if self.free_left:
                self.get_logger().info("Evitando por la izquierda.")
                drive_msg.drive.steering_angle = 0.4
            elif self.free_right:
                self.get_logger().info("Evitando por la derecha.")
                drive_msg.drive.steering_angle = -0.4
            else:
                self.get_logger().warn("¡Sin espacio para evitar! Deteniendo vehículo.")
                drive_msg.drive.speed = 0.0
                drive_msg.drive.steering_angle = 0.0

            self.drive_pub.publish(drive_msg)
            return

        # Seguimiento normal por Pure Pursuit
        target = self.find_lookahead_point()
        if target is None:
            self.get_logger().warn('No se encontró punto de lookahead. Deteniendo vehículo.')
            stop_msg = AckermannDriveStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'base_link'
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(stop_msg)
            return

        # Publicar lookahead point para visualización
        self.lookahead_pub.publish(target)

        # Publicar lookahead line para visualización
        self.publish_lookahead_line(self.current_pose.position, target.pose.position)

        # Transformar target point a local frame
        dx = target.pose.position.x - self.current_pose.position.x
        dy = target.pose.position.y - self.current_pose.position.y
        distance_to_target = math.sqrt(dx**2 + dy**2)

        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        local_x = math.cos(-yaw)*dx - math.sin(-yaw)*dy
        local_y = math.sin(-yaw)*dx + math.cos(-yaw)*dy

        curvature = (2 * local_y) / (self.lookahead_distance ** 2)
        angle = math.atan(curvature * 2.5)  # 2.5 es una distancia entre ejes típica

         # Lógica de velocidad adaptativa
        if distance_to_target < 1.0:
            speed = 0.5  # reducir velocidad cerca del objetivo
        else:
            speed = 2.0

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed  # m/s
        self.drive_pub.publish(drive_msg)

        # Detectar final de trayectoria, calcular duración y parar vehículo
        if self.reached_goal(target) and not self.trajectory_completed:
            self.end_time = self.get_clock().now()
            duration = (self.end_time - self.start_time).nanoseconds / 1e9
            self.get_logger().info(f'Trayectoria completada en {duration:.2f} segundos')

            # Publicar mensaje de parada
            stop_msg = AckermannDriveStamped()
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            stop_msg.header.frame_id = 'base_link'
            stop_msg.drive.speed = 0.0
            stop_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(stop_msg)

            # Cambiar el flag
            self.trajectory_completed = True
            self.active = False
            return

    def reached_goal(self, lookahead):
        goal = self.trajectory[-1].pose.position
        dx = self.current_pose.position.x - goal.x
        dy = self.current_pose.position.y - goal.y
        distance = math.sqrt(dx**2 + dy**2)
        return distance < 0.5  # tolerancia

    def find_lookahead_point(self):
        if not self.trajectory:
            return None

        closest_dist = float('inf')
        closest_idx = 0

        # Paso 1: encontrar el punto más cercano
        for i, pose in enumerate(self.trajectory):
            dx = pose.pose.position.x - self.current_pose.position.x
            dy = pose.pose.position.y - self.current_pose.position.y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        # Paso 2: buscar desde el punto más cercano hacia adelante
        for pose in self.trajectory[closest_idx:]:
            dx = pose.pose.position.x - self.current_pose.position.x
            dy = pose.pose.position.y - self.current_pose.position.y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > self.lookahead_distance:
                # Asegurar que el punto de avance esté en el frame 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                return pose
            
        # Si no se encuentra un punto de avance, forzar el último punto como objetivo final
        final_pose = self.trajectory[-1]
        dx = final_pose.pose.position.x - self.current_pose.position.x
        dy = final_pose.pose.position.y - self.current_pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > 0.2:  # Sólo si aún no hemos llegado completamente
            final_pose.header.stamp = self.get_clock().now().to_msg()
            final_pose.header.frame_id = 'map'
            return final_pose

        return None # Si no se encuentra un punto de avance, ya está cerca y no hace falta seguir

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_lookahead_line(self, start, end):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lookahead_line'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # grosor de línea
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0  # permanece hasta que se sobreescriba

        p1 = Point()
        p1.x = start.x
        p1.y = start.y
        p1.z = 0.0

        p2 = Point()
        p2.x = end.x
        p2.y = end.y
        p2.z = 0.0

        marker.points = [p1, p2]
        self.line_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
