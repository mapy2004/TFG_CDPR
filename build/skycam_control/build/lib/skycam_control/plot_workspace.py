import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import linprog
from scipy.spatial import ConvexHull
import sys

def compute_catenary_wfw():
    mass = 25.0
    g = 9.81
    W_grav = np.array([0, 0, mass * g]) 
    
    anchors = np.array([
        [ 30.0,  20.0, 30.0], [ 30.0, -20.0, 30.0],
        [-30.0,  20.0, 30.0], [-30.0, -20.0, 30.0]
    ])
    
    val = 0.45
    body_points = np.array([
        [ val,  val, 0.05], [ val, -val, 0.05], 
        [-val,  val, 0.05], [-val, -val, 0.05]  
    ])

    # --- LÍMITES REALES ---
    T_max = 800.0  
    L_min = 1.5    

    # --- MODELO DE CATENARIA (El peso del cable) ---
    rho = 0.05      # Densidad lineal del cable (50 gramos por metro, típico en Skycams)
    max_sag = 0.6   # Permitimos un pandeo máximo de 60 centímetros en el centro del cable

    x_vals = np.linspace(-29, 29, 30)
    y_vals = np.linspace(-19, 19, 30)
    z_vals = np.linspace(1, 30, 25)

    feasible_points = []
    total_iterations = len(x_vals) * len(y_vals) * len(z_vals)
    current_iter = 0

    print("Calculando WFW con Física de Catenaria (Cable Sagging)...")

    for z in z_vals:
        for y in y_vals:
            for x in x_vals:
                current_iter += 1
                if current_iter % 1000 == 0:
                    progreso = (current_iter / total_iterations) * 100
                    sys.stdout.write(f"\rProgreso: {progreso:.1f}% ({current_iter}/{total_iterations})")
                    sys.stdout.flush()

                current_pos = np.array([x, y, z])
                J = np.zeros((3, 4))
                valid_point = True
                
                # Lista para guardar el T_min de cada cable basado en su longitud
                t_min_bounds = []
                
                for i in range(4):
                    b_global = current_pos + body_points[i] 
                    L = anchors[i] - b_global
                    dist = np.linalg.norm(L)
                    
                    if dist < L_min: 
                        valid_point = False
                        break
                        
                    u = L / dist
                    J[0:3, i] = u 
                    
                    # FÍSICA APLICADA: Calcular T_min para que este cable no se combe
                    L_xy = np.linalg.norm(L[0:2]) # Proyección horizontal
                    T_sag = (rho * g * L_xy**2) / (8 * max_sag)
                    
                    # El cable debe tener al menos 10N, o la fuerza de la catenaria (la mayor de las dos)
                    t_min_bounds.append(max(10.0, T_sag))
                
                if not valid_point:
                    continue
                
                # Montamos los límites personalizados para cada cable
                bounds = [(t_min_bounds[i], T_max) for i in range(4)]
                
                # Si algún T_min ya supera los 800N, es físicamente imposible
                if any(t > T_max for t in t_min_bounds):
                    continue
                
                res = linprog(np.ones(4), A_eq=J, b_eq=W_grav, bounds=bounds, method='highs')
                
                if res.success:
                    feasible_points.append([x, y, z])

    points = np.array(feasible_points)
    print(f"\n¡Completado! Puntos factibles: {len(points)}")

    # --- RENDERIZADO ---
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    hull = ConvexHull(points)
    
    surf = ax.plot_trisurf(points[:,0], points[:,1], points[:,2], 
                           triangles=hull.simplices, cmap='viridis', 
                           alpha=0.8, edgecolor='black', linewidth=0.2)
    
    for a in anchors:
        ax.plot([a[0], a[0]], [a[1], a[1]], [0, a[2]], color='gray', linewidth=2, linestyle='--')
        ax.scatter(a[0], a[1], a[2], color='red', s=60, marker='^', zorder=5)

    ax.set_xlabel('Eje X (m)')
    ax.set_ylabel('Eje Y (m)')
    ax.set_zlabel('Altura Z (m)')
    ax.set_xlim(-35, 35)
    ax.set_ylim(-25, 25)
    ax.set_zlim(0, 35)
    ax.set_box_aspect([70, 50, 35]) 
    
    ax.set_title('WFW Dinámico con Restricción de Catenaria (Sagging)\n$T_{max}=800$N, Pandeo máx. permitido = 0.6m', 
                 fontsize=14, weight='bold', pad=20)
    
    mappable = plt.cm.ScalarMappable(cmap='viridis')
    mappable.set_array(points[:,2])
    cbar = fig.colorbar(mappable, shrink=0.5, pad=0.1)
    cbar.set_label('Altura Operativa (m)', rotation=270, labelpad=20)
    
    ax.view_init(elev=20, azim=45)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    compute_catenary_wfw()