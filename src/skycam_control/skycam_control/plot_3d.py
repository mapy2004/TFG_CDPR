import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    # Leer el archivo generado por Gazebo/ROS2
    try:
        df = pd.read_csv('trajectory_log.csv')
    except FileNotFoundError:
        print("No se encuentra 'trajectory_log.csv'. Asegúrate de simular un vuelo primero.")
        return

    # Extraemos los datos a arrays de NumPy puros para evitar el error de Pandas
    t_x = df['target_x'].to_numpy()
    t_y = df['target_y'].to_numpy()
    t_z = df['target_z'].to_numpy()
    
    a_x = df['actual_x'].to_numpy()
    a_y = df['actual_y'].to_numpy()
    a_z = df['actual_z'].to_numpy()

    # Crear la figura
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plotear la ruta deseada (Lookahead) - Línea punteada azul transparente
    ax.plot(t_x, t_y, t_z, 
            label='Trayectoria Deseada (Lookahead)', 
            color='blue', linestyle='--', linewidth=1.5, alpha=0.5)

    # Plotear la ruta real de la cámara (Ground Truth) - Línea sólida roja
    ax.plot(a_x, a_y, a_z, 
            label='Trayectoria Real (Odometría)', 
            color='red', linewidth=2.5)

    # Puntos de inicio y fin para dar contexto
    ax.scatter(a_x[0], a_y[0], a_z[0], color='green', s=100, label='Inicio Vuelo', marker='o')
    ax.scatter(a_x[-1], a_y[-1], a_z[-1], color='black', s=100, label='Fin Vuelo', marker='X')

    # Etiquetas y título
    ax.set_xlabel('Eje X (m)', fontsize=12)
    ax.set_ylabel('Eje Y (m)', fontsize=12)
    ax.set_zlabel('Altura Z (m)', fontsize=12)
    ax.set_title('Rendimiento del Seguimiento de Trayectoria 3D (CDPR)', fontsize=16, fontweight='bold')

    # Ajustar límites para que se vea como en tu WFW (Zoom en la zona de 15m)
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(14, 16) 

    ax.legend(loc='upper right', fontsize=11)
    plt.tight_layout()

    # Guardar y mostrar
    ax.view_init(elev=90, azim=-90) # Fuerzas la cámara desde arriba
    plt.savefig('Figure_trayectoria_3d.png', dpi=300)
    print("¡Gráfica guardada con éxito como Figure_trayectoria_3d.png!")
    plt.show()

if __name__ == '__main__':
    main()