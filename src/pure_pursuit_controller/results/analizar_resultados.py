import os
import csv
import matplotlib.pyplot as plt # Asegúrate de tener matplotlib instalado

# Ruta actual donde están los ficheros
current_dir = os.path.dirname(os.path.abspath(__file__))

# Buscar archivos CSV en el mismo directorio 
csv_files = [f for f in os.listdir(current_dir) if f.endswith('.csv') and f.startswith('results_')]

# Variables para gráficos 
timestamps = []
rmses = []

# Leer datos de todos los archivos 
for filename in sorted(csv_files):
    filepath = os.path.join(current_dir, filename)
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rmse = float(row['rmse'])
            if rmse > 0.0:  # Ignorar 0s
                rmses.append(rmse)
                timestamps.append(float(row['timestamp']))

# Comprobar si hay datos 
if not rmses:
    print("No hay datos de RMSE válidos para graficar.")
    exit()

# Gráfico 1: RMSE vs Tiempo 
plt.figure(figsize=(14, 6))
plt.plot(
    timestamps, 
    rmses, 
    marker='o',
    markersize=4,
    markerfacecolor='none',
    linestyle='-',
    color='blue', 
    label='RMSE')
plt.title('RMSE frente al tiempo')
plt.xlabel('Tiempo (s)')
plt.ylabel('RMSE')
plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(current_dir, 'grafica_rmse_vs_tiempo.png'))
plt.close()

# Gráfico 2: Histograma horizontal (min, media, max)
rmse_min = min(rmses)
rmse_max = max(rmses)
rmse_mean = sum(rmses) / len(rmses)

metricas = ['Mínimo', 'Media', 'Máximo']
valores = [rmse_min, rmse_mean, rmse_max]

plt.figure(figsize=(10, 5))
bars = plt.barh(metricas, valores, color=['green', 'orange', 'red'])
plt.xlabel('RMSE')
plt.title('Estadísticas de RMSE')
plt.grid(True, axis='x', which='both', linestyle='--', linewidth=0.5)

# Agregar valores numéricos al lado de cada barra
for bar in bars:
    width = bar.get_width()
    plt.text(width + 0.0005, bar.get_y() + bar.get_height() / 2,
             f'{width:.5f}', va='center')

plt.tight_layout()
plt.savefig(os.path.join(current_dir, 'grafica_estadisticas_rmse.png'))
plt.close()

print("Gráficas guardadas exitosamente en:")
print(f" - grafica_rmse_vs_tiempo.png")
print(f" - grafica_estadisticas_rmse.png")
