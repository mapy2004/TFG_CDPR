import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import nnls

def compute_tensions_3d():
    mass = 25.0
    g = 9.81
    anchors = np.array([
        [ 10.0,  10.0, 15.0], [ 10.0, -10.0, 15.0],
        [-10.0,  10.0, 15.0], [-10.0, -10.0, 15.0]
    ])
    val = 0.45
    body_points = np.array([
        [ val,  val, 0.05], [ val, -val, 0.05], 
        [-val,  val, 0.05], [-val, -val, 0.05]  
    ])
    com_offset_body = np.array([0.0, 0.0, -0.15])
    
    T_min = 10.0
    T_max = 4000.0

    # Crear malla 2D para evaluar el plano XY a la altura de Z=6
    res = 20 # Resolución de la malla
    x_range = np.linspace(-9, 9, res)
    y_range = np.linspace(-9, 9, res)
    X, Y = np.meshgrid(x_range, y_range)
    Z_val = 6.0

    # Matrices para almacenar las tensiones de los 4 cables
    T1 = np.zeros_like(X)
    T2 = np.zeros_like(X)
    T3 = np.zeros_like(X)
    T4 = np.zeros_like(X)
    
    Weights = np.diag([1.0, 1.0, 3.0, 0.01, 0.01, 0.1])
    W_desired = np.array([0, 0, mass * g, 0, 0, 0])

    print("Calculando superficies de tensión 3D...")
    for i in range(res):
        for j in range(res):
            current_pos = np.array([X[i,j], Y[i,j], Z_val])
            J = np.zeros((6, 4))
            for k in range(4):
                r_body = body_points[k] - com_offset_body
                b_global = current_pos + body_points[k]
                L = anchors[k] - b_global
                dist = np.linalg.norm(L)
                u = L / dist if dist > 0.01 else np.array([0,0,1])
                J[0:3, k] = u
                J[3:6, k] = np.cross(r_body, u)

            J_weighted = Weights @ J
            W_weighted = Weights @ W_desired
            T_min_vec = np.full(4, T_min)
            W_adjusted = W_weighted - (J_weighted @ T_min_vec)

            try:
                delta_tensions, _ = nnls(J_weighted, W_adjusted)
                tensions = delta_tensions + T_min_vec
                tensions = np.clip(tensions, T_min, T_max)
            except:
                tensions = np.array([np.nan]*4)
                
            T1[i,j], T2[i,j], T3[i,j], T4[i,j] = tensions

    # Visualización
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle(f'Distribución Espacial de Tensiones a Z={Z_val}m', fontsize=16)

    cables = [T1, T2, T3, T4]
    titles = ['Cable 1 (+X, +Y)', 'Cable 2 (+X, -Y)', 'Cable 3 (-X, +Y)', 'Cable 4 (-X, -Y)']

    for i in range(4):
        ax = fig.add_subplot(2, 2, i+1, projection='3d')
        surf = ax.plot_surface(X, Y, cables[i], cmap='jet', edgecolor='none', alpha=0.8)
        ax.set_title(titles[i])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Tensión (N)')
        ax.set_zlim(0, np.nanmax(cables))
        fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    compute_tensions_3d()