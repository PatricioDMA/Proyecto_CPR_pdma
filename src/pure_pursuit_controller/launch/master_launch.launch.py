from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Parámetros desde terminal
    use_noisy_odom_arg = DeclareLaunchArgument(
        'use_noisy_odom', default_value='false', description='Activar odometría con ruido')
    use_drive_raw_arg = DeclareLaunchArgument(
        'use_drive_raw', default_value='false', description='Activar publicación en /drive_raw')
    use_latency_arg = DeclareLaunchArgument(
        'use_latency', default_value='false', description='Activar modo con latencia (frecuencia de control reducida)')

    use_noisy_odom = LaunchConfiguration('use_noisy_odom')
    use_drive_raw = LaunchConfiguration('use_drive_raw')
    use_latency = LaunchConfiguration('use_latency')

    # Ruta al launch del simulador
    f1tenth_gym_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('f1tenth_gym_ros'),
                'launch',
                'gym_bridge_launch.py'
            )
        ])
    )

    # Nodo de trayectoria
    trajectory_node = Node(
        package='trajectory_agent',
        executable='trajectory_node',
        name='trajectory_node',
        output='screen'
    )

    # Nodo de control con parámetros
    control_node = Node(
        package='pure_pursuit_controller',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'use_noisy_odom': use_noisy_odom,
            'use_drive_raw': use_drive_raw,
            'use_latency': use_latency
        }]
    )

    # Nodo odometría con ruido (solo si use_noisy_odom = true)
    noisy_odom_node = Node(
        condition=IfCondition(use_noisy_odom),
        package='odom_modifiers',
        executable='noisy_odom_publisher',
        name='noisy_odom_publisher',
        output='screen'
    )

    # Nodo de error sistemático en dirección (solo si use_drive_raw = true)
    steering_bias_node = Node(
        condition=IfCondition(use_drive_raw),
        package='odom_modifiers',
        executable='steering_bias_node',
        name='steering_bias_node',
        output='screen'
    )

    # Retornar la descripción del lanzamiento
    return LaunchDescription([
        use_noisy_odom_arg,
        use_drive_raw_arg,
        use_latency_arg,
        f1tenth_gym_launch,
        trajectory_node,
        control_node,
        noisy_odom_node,
        steering_bias_node
    ])
