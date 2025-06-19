import os
import csv
import matplotlib.pyplot as plt

# Etiquetas esperadas (puedes ajustar el orden si cambia)
etiquetas = [
    "Sin errores",
    "Odometría ruidosa",
    "Sesgo en dirección",
    "Con latencia"
]

# Colores distintos para cada serie
colores = ['blue', 'orange', 'green', 'red']

# Buscar los ficheros CSV en el directorio actual
current_dir = os.path.dirname(os.path.abspath(__file__))
csv_files = sorted([f for f in os.listdir(current_dir) if f.endswith('.csv') and f.startswith('results_')])

if len(csv_files) != 4:
    print("AVISO: Se esperaban exactamente 4 archivos 'results_*.csv'. Encontrados:", len(csv_files))
    exit()

# Inicializar la figura
plt.figure(figsize=(14, 6))

# Procesar cada archivo
for i, filename in enumerate(csv_files):
    filepath = os.path.join(current_dir, filename)
    timestamps = []
    rmses = []

    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rmse = float(row['rmse'])
            if rmse > 0.0:
                timestamps.append(float(row['timestamp']))
                rmses.append(rmse)

    # Añadir la curva al gráfico
    plt.plot(
        timestamps,
        rmses,
        marker='o',
        markersize=4,
        markerfacecolor='none',
        linestyle='-',
        color=colores[i],
        label=etiquetas[i]
    )

# Decoración
plt.title('Comparación de RMSE frente al tiempo')
plt.xlabel('Tiempo (s)')
plt.ylabel('RMSE')
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.legend()
plt.tight_layout()

# Guardar
output_path = os.path.join(current_dir, 'grafica_comparativa_rmse.png')
plt.savefig(output_path)
plt.close()

print("Gráfica comparativa guardada exitosamente en:")
print(f" - {output_path}")
