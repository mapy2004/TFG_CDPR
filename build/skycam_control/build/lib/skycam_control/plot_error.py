import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_tracking_error(csv_file):
    try:
        data = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: No se encuentra el archivo {csv_file}")
        return

    tiempo = data['Tiempo_s'].to_numpy()
    error = data['Error_Pixeles'].to_numpy()

    # Configuración de la gráfica 
    plt.figure(figsize=(12, 5))
    
    # Dibujar la línea de error
    plt.plot(tiempo, error, label='Error de seguimiento', color='#9467bd', linewidth=2)

    # Dibujar la zona de tolerancia estricta (Deadband de 5 píxeles)
    plt.axhline(5, color='green', linestyle='--', linewidth=2, label='Límite Deadband (5 px)')
    plt.fill_between(tiempo, 0, 5, color='green', alpha=0.15, label='Zona de Encuadre Perfecto')

    # Línea de media para análisis estadístico
    media_error = np.mean(error)
    plt.axhline(media_error, color='orange', linestyle=':', linewidth=2, label=f'Error Medio ({media_error:.1f} px)')

    # Formato académico
    plt.title('Rendimiento del Seguimiento Visual (Visual Servoing) en Vuelo Dinámico', fontsize=14, weight='bold', pad=15)
    plt.xlabel('Tiempo de vuelo (s)', fontsize=12)
    plt.ylabel('Distancia al centro (Píxeles)', fontsize=12)
    
    # Ajustar ejes
    plt.xlim(0, max(tiempo))
    plt.ylim(0, max(max(error) + 10, 50)) # Altura dinámica pero al menos 50 px

    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend(loc='upper right', framealpha=0.9)
    plt.tight_layout()

    # Mostrar la gráfica
    plt.show()

def main(args=None):
    # La ruta al CSV asume que ejecutas el script desde la carpeta donde está el archivo.
    # Si te da error de "No se encuentra el archivo", pon la ruta absoluta, 
    # por ejemplo: '/home/mapy/skycam_ws/error_seguimiento.csv'
    plot_tracking_error('error_seguimiento.csv')

if __name__ == '__main__':
    main()