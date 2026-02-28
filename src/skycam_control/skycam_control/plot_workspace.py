import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import linprog

def compute_feasible_workspace():
    mass = 25.0
    g = 9.81
    W_grav = np.array([0, 0, mass * g, 0, 0, 0]) 
    
    anchors = np.array([
        [ 10.0,  10.0, 15.0], [ 10.0, -10.0, 15.0],
        [-10.0,  10.0, 15.0], [-10.0, -10.0, 15.0]
    ])
    val = 0.45
    body_points = np.array([
        [ val,  val, 0.05], [ val, -val, 0.05], 
        [-val,  val, 0.05], [-val, -val, 0.05]  
    ])
    
    # MEJORA 1: Bajamos el Centro de Masas a 80 cm por debajo de los enganches
    # Esto le da una estabilidad pendular masiva.
    com_offset_body = np.array([0.0, 0.0, -0.8])

    T_min = 15.0
    T_max = 4000.0
    
    # MEJORA 2: Aumentamos la tolerancia del Gimbal/Torque a 50 Nm
    m_max = 50.0 

    x_vals = np.linspace(-9, 9, 25)
    y_vals = np.linspace(-9, 9, 25)
    z_vals = np.linspace(1, 14, 20)

    feasible_points = []
    
    print("Calculando Wrench-Feasible Workspace ampliado... ")

    for z in z_vals:
        for y in y_vals:
            for x in x_vals:
                current_pos = np.array([x, y, z])
                R_mat = np.eye(3) 
                
                J = np.zeros((6, 4))
                for i in range(4):
                    r_body = body_points[i] - com_offset_body
                    b_global = current_pos + (R_mat @ body_points[i])
                    L = anchors[i] - b_global
                    dist = np.linalg.norm(L)
                    u = L / dist if dist > 1e-3 else np.array([0,0,1])
                    
                    J[0:3, i] = u
                    J[3:6, i] = np.cross(r_body, u)
                
                A_eq = J[[0, 1, 2, 5], :]
                b_eq = W_grav[[0, 1, 2, 5]]
                
                A_ub = np.vstack([J[3, :], -J[3, :], J[4, :], -J[4, :]])
                b_ub = np.array([m_max, m_max, m_max, m_max])
                
                bounds = [(T_min, T_max) for _ in range(4)]
                
                res = linprog(np.ones(4), A_ub=A_ub, b_ub=b_ub, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method='highs')
                
                if res.success:
                    feasible_points.append([x, y, z])

    feasible_points = np.array(feasible_points)
    print(f"Puntos factibles encontrados: {len(feasible_points)}")
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    p = ax.scatter(feasible_points[:,0], feasible_points[:,1], feasible_points[:,2], 
                   c=feasible_points[:,2], cmap='viridis', alpha=0.6, s=20)
    
    for a in anchors:
        ax.plot([a[0], a[0]], [a[1], a[1]], [0, a[2]], 'k-', linewidth=2, alpha=0.5)
        ax.scatter(a[0], a[1], a[2], color='red', s=50, label='Anclajes' if a[0]==10 and a[1]==10 else "")

    ax.set_xlabel('Eje X (m)')
    ax.set_ylabel('Eje Y (m)')
    ax.set_zlabel('Eje Z (m)')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(0, 15)
    ax.set_title('Wrench-Feasible Workspace Mejorado (CoM -0.8m, Torque $\leq 50$ Nm)')
    fig.colorbar(p, label='Altura Z (m)', shrink=0.7)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    compute_feasible_workspace()