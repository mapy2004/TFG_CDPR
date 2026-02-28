#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from enum import Enum

# --- 1. MÁQUINA DE ESTADOS FINITOS (MFS) ---
class State(Enum):
    IDLE = 0
    APPROACHING_START = 1
    TRACKING = 2
    DONE = 3

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('skycam_trajectory_planner')
        
        # Publicador de la posición objetivo hacia el controlador de bajo nivel
        self.target_pub = self.create_publisher(Point, '/skycam/target_pos', 10)
        self.odom_sub = self.create_subscription(Odometry, '/skycam/platform_odom', self.odom_cb, 10)
        
        self.current_pos = None
        
        # Parámetros de la MFS
        self.state = State.IDLE
        self.approach_tolerance = 0.15  # Metros de error permitidos para cambiar de APPROACH a TRACKING
        
        # --- PARÁMETROS DE LAS FÓRMULAS ---
        # Lookahead (Pure Pursuit 3D)
        self.lookahead_distance = 0.8  # L_d: Distancia de visión hacia adelante (metros)
        
        # Control Dinámico de Velocidad
        self.v_max = 2.5       # Velocidad máxima en rectas (m/s)
        self.v_min = 0.5       # Velocidad mínima en curvas cerradas (m/s)
        self.beta = 2.0        # Sensibilidad a la curvatura (freno paramétrico)
        
        # Generación de la ruta
        self.waypoints = self.generate_figure_eight_path()
        self.curvatures = self.calculate_discrete_curvature(self.waypoints)
        
        self.current_target_idx = 0
        
        # Bucle de control a 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('Planificador de Trayectorias Iniciado.')

    def generate_figure_eight_path(self):
        """ Genera una trayectoria suave paramétrica para evitar cambios bruscos de tensión """
        t = np.linspace(0, 2 * np.pi, 500)
        R_x, R_y = 6.0, 4.0  # Radios de la pista
        Z_height = 6.0
        
        path = np.zeros((len(t), 3))
        path[:, 0] = R_x * np.sin(t)
        path[:, 1] = R_y * np.sin(2 * t)
        path[:, 2] = Z_height
        return path

    def calculate_discrete_curvature(self, path):
        """ 2. FÓRMULA DE CURVATURA: k = ||r' x r''|| / ||r'||^3 """
        curvatures = np.zeros(len(path))
        for i in range(1, len(path) - 1):
            # Derivadas por diferencias finitas (aproximación discreta)
            r_prime = (path[i+1] - path[i-1]) / 2.0
            r_double_prime = path[i+1] - 2*path[i] + path[i-1]
            
            norm_r_prime = np.linalg.norm(r_prime)
            if norm_r_prime > 1e-5:
                cross_prod = np.cross(r_prime, r_double_prime)
                curvatures[i] = np.linalg.norm(cross_prod) / (norm_r_prime ** 3)
            else:
                curvatures[i] = 0.0
                
        # Igualar extremos
        curvatures[0] = curvatures[1]
        curvatures[-1] = curvatures[-2]
        return curvatures

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        if not np.isnan(p.x):
            self.current_pos = np.array([p.x, p.y, p.z])
            # Iniciar MFS automáticamente al recibir la primera posición
            if self.state == State.IDLE:
                self.state = State.APPROACHING_START
                self.get_logger().info('MFS: Transición a APPROACHING_START')

    def control_loop(self):
        if self.current_pos is None:
            return

        target = Point()

        # --- LÓGICA DE LA MÁQUINA DE ESTADOS FINITOS (MFS) ---
        if self.state == State.APPROACHING_START:
            # Apuntar al primer waypoint de la trayectoria
            start_point = self.waypoints[0]
            target.x, target.y, target.z = start_point[0], start_point[1], start_point[2]
            
            # Condición de transición
            error = np.linalg.norm(start_point - self.current_pos)
            if error < self.approach_tolerance:
                self.state = State.TRACKING
                self.get_logger().info('MFS: Tolerancia alcanzada. Transición a TRACKING')

        elif self.state == State.TRACKING:
            # Buscar el punto más cercano en la trayectoria
            distances = np.linalg.norm(self.waypoints - self.current_pos, axis=1)
            closest_idx = np.argmin(distances)
            
            # 3. CONTROL DINÁMICO DE VELOCIDAD basado en la curvatura actual
            kappa = self.curvatures[closest_idx]
            v_cmd = self.v_max / (1.0 + self.beta * kappa)
            v_cmd = np.clip(v_cmd, self.v_min, self.v_max)
            
            # 4. FÓRMULA DEL LOOKAHEAD (Pure Pursuit)
            # Adaptamos dinámicamente la distancia de visión según la velocidad
            dynamic_lookahead = self.lookahead_distance * (v_cmd / self.v_max)
            
            lookahead_idx = closest_idx
            for i in range(closest_idx, len(self.waypoints)):
                dist = np.linalg.norm(self.waypoints[i] - self.current_pos)
                if dist >= dynamic_lookahead:
                    lookahead_idx = i
                    break
            
            # Si llegamos al final del array
            if lookahead_idx >= len(self.waypoints) - 1:
                lookahead_idx = len(self.waypoints) - 1
                dist_to_end = np.linalg.norm(self.waypoints[-1] - self.current_pos)
                if dist_to_end < self.approach_tolerance:
                    self.state = State.DONE
                    self.get_logger().info('MFS: Trayectoria completada. Transición a DONE')
            
            # Extraer coordenadas objetivo
            p_target = self.waypoints[lookahead_idx]
            target.x, target.y, target.z = p_target[0], p_target[1], p_target[2]

        elif self.state == State.DONE:
            # Mantener la posición final
            end_point = self.waypoints[-1]
            target.x, target.y, target.z = end_point[0], end_point[1], end_point[2]

        # Publicar el comando hacia el controlador de bajo nivel
        if self.state != State.IDLE:
            self.target_pub.publish(target)

def main():
    rclpy.init()
    node = TrajectoryPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()