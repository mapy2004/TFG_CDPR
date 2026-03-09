import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_flight_tensions(csv_file):
    # Leer los datos del CSV
    try:
        data = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: No se encuentra el archivo {csv_file}")
        return

    tiempo = data['Tiempo_s'].to_numpy()
    t1 = data['Tension_1_N'].to_numpy()
    t2 = data['Tension_2_N'].to_numpy()
    t3 = data['Tension_3_N'].to_numpy()
    t4 = data['Tension_4_N'].to_numpy()

    # Configuración de la gráfica 
    plt.figure(figsize=(12, 6))
    
    plt.plot(tiempo, t1, label='Cable 1 (T1)', color='#d62728', linewidth=1.5) # Rojo
    plt.plot(tiempo, t2, label='Cable 2 (T2)', color='#1f77b4', linewidth=1.5) # Azul
    plt.plot(tiempo, t3, label='Cable 3 (T3)', color='#2ca02c', linewidth=1.5) # Verde
    plt.plot(tiempo, t4, label='Cable 4 (T4)', color='#ff7f0e', linewidth=1.5) # Naranja

    # Límites operativos (Estático y Dinámico)
    plt.axhline(800, color='black', linestyle='--', linewidth=2, label='Límite Superior ($T_{max} = 800$ N)')
    plt.axhline(10, color='gray', linestyle='-.', linewidth=2, label='Límite Inferior Estructural ($T_{min} = 10$ N)')

    # Sombrear la zona de "peligro" por encima de 800N para que quede claro que nunca se entra ahí
    plt.fill_between(tiempo, 800, 850, color='red', alpha=0.1)

    # Formato académico
    plt.title('Evolución de las Tensiones de los Cables en Vuelo Dinámico', fontsize=15, weight='bold', pad=15)
    plt.xlabel('Tiempo de vuelo (s)', fontsize=12)
    plt.ylabel('Tensión (N)', fontsize=12)
    
    # Ajustar ejes
    plt.xlim(0, max(tiempo))
    # Damos un poco de margen por arriba y por abajo para ver bien los límites
    max_y = max(820, max(max(t1), max(t2), max(t3), max(t4)) + 50)
    plt.ylim(0, max_y)

    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend(loc='upper right', framealpha=0.9)
    plt.tight_layout()

    # Mostrar la gráfica
    plt.show()

if __name__ == '__main__':
    # Asegúrate de que el nombre coincide con tu archivo guardado
    plot_flight_tensions('tensiones_vuelo.csv')