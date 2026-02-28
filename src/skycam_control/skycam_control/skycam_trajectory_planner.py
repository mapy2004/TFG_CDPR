#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from enum import Enum

class State(Enum):
    IDLE = 0
    APPROACHING_START = 1
    TRACKING = 2
    DONE = 3
    TELEOP = 4  # NUEVO ESTADO: Teleoperación manual

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('skycam_trajectory_planner')
        
        self.target_pub = self.create_publisher(Point, '/skycam/target_pos', 10)
        self.odom_sub = self.create_subscription(Odometry, '/skycam/platform_odom', self.odom_cb, 10)
        
        # NUEVO: Suscriptor para el joystick/teclado
        self.cmd_sub = self.create_subscription(Twist, '/skycam/cmd_vel', self.cmd_cb, 10)
        
        self.current_pos = None
        self.state = State.IDLE
        self.approach_tolerance = 0.15 
        
        # --- PARÁMETROS DE TRAYECTORIA QUÍNTICA (Modo Automático) ---
        self.T_total = 25.0  
        self.t_current = 0.0
        self.dt = 0.02       
        self.R_x = 6.0
        self.R_y = 4.0
        self.Z_height = 6.0
        self.time_lookahead = 0.3 
        
        # --- PARÁMETROS TELEOP Y FÓRMULAS ---
        self.teleop_target = np.array([0.0, 0.0, 6.0]) # Punto virtual inicial
        self.manual_vel = np.zeros(3)
        self.last_manual_vel = np.zeros(3)
        self.v_max = 5.0       # Velocidad máxima permitida por el operador
        self.v_min = 0.5
        self.beta = 2.0        # Factor de freno en giros bruscos
        self.lookahead_distance = 0.5 # Distancia de proyección
        
        # --- SEGURIDAD WFW (CLAMPING) ---
        self.safe_x_limit = 8.0  
        self.safe_y_limit = 8.0  
        self.safe_z_min = 2.0    
        self.safe_z_max = 13.0   

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Planificador Iniciado. Modos disponibles: AUTOMÁTICO (Lemniscata) y TELEOP (cmd_vel).')

    def get_quintic_scaling(self, t, T):
        if t <= 0: return 0.0
        if t >= T: return 1.0
        tau = t / T
        return 10*(tau**3) - 15*(tau**4) + 6*(tau**5)

    def evaluate_path(self, s):
        theta = s * 2.0 * np.pi
        x = self.R_x * np.sin(theta)
        y = self.R_y * np.sin(2 * theta)
        z = self.Z_height
        return np.array([x, y, z])

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        if not np.isnan(p.x):
            self.current_pos = np.array([p.x, p.y, p.z])
            # Inicio automático solo si no estamos ya en otro estado
            if self.state == State.IDLE:
                self.state = State.APPROACHING_START
                self.get_logger().info('MFS: Iniciando secuencia automática (APPROACHING_START)')

    def cmd_cb(self, msg):
        """ Recibe comandos de velocidad del operador """
        self.manual_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
        
        # Si se recibe cualquier comando, forzamos la MFS al modo TELEOP
        if np.linalg.norm(self.manual_vel) > 0.01 and self.state != State.TELEOP:
            self.state = State.TELEOP
            # Sincronizamos el punto virtual con la posición real para no dar tirones
            if self.current_pos is not None:
                self.teleop_target = self.current_pos.copy()
            self.get_logger().info('MFS: Override manual detectado. Transición a TELEOP.')

    def control_loop(self):
        if self.current_pos is None: return

        target = Point()
        active_command = False

        if self.state == State.APPROACHING_START:
            start_point = self.evaluate_path(0.0)
            target.x, target.y, target.z = start_point[0], start_point[1], start_point[2]
            active_command = True
            
            error = np.linalg.norm(start_point - self.current_pos)
            if error < self.approach_tolerance:
                self.state = State.TRACKING
                self.t_current = 0.0
                self.get_logger().info('MFS: Posición alcanzada. Iniciando TRACKING Quíntico.')

        elif self.state == State.TRACKING:
            self.t_current += self.dt
            t_target = min(self.t_current + self.time_lookahead, self.T_total)
            s = self.get_quintic_scaling(t_target, self.T_total)
            p_target = self.evaluate_path(s)
            
            target.x, target.y, target.z = p_target[0], p_target[1], p_target[2]
            active_command = True
            
            if self.t_current >= self.T_total:
                self.state = State.DONE
                self.get_logger().info('MFS: Trayectoria finalizada. Transición a DONE')

        elif self.state == State.DONE:
            end_point = self.evaluate_path(1.0)
            target.x, target.y, target.z = end_point[0], end_point[1], end_point[2]
            active_command = True

        elif self.state == State.TELEOP:
            # --- 1. CÁLCULO DE CURVATURA INSTANTÁNEA ---
            # Evaluamos la intención de giro comparando la velocidad actual con la anterior
            accel_intent = (self.manual_vel - self.last_manual_vel) / self.dt
            vel_norm = np.linalg.norm(self.manual_vel)
            
            kappa = 0.0
            if vel_norm > 0.1:
                cross_prod = np.cross(self.manual_vel, accel_intent)
                kappa = np.linalg.norm(cross_prod) / (vel_norm ** 3)
            
            self.last_manual_vel = self.manual_vel.copy()

            # --- 2. CONTROL DINÁMICO DE VELOCIDAD ---
            # Frenamos el comando del usuario si intenta hacer un giro muy cerrado a alta velocidad
            speed_factor = 1.0 / (1.0 + self.beta * kappa)
            safe_vel = self.manual_vel * speed_factor
            
            # --- 3. FÓRMULA DEL LOOKAHEAD (Integración Cinemática) ---
            # En lugar de enviar la velocidad pura, avanzamos el "Punto Objetivo" (Target)
            # a una distancia proporcional, logrando un movimiento guiado y elástico.
            self.teleop_target += safe_vel * self.dt
            
            target.x, target.y, target.z = self.teleop_target[0], self.teleop_target[1], self.teleop_target[2]
            active_command = True

        # --- APLICACIÓN DE SEGURIDAD WFW (CLAMPING) ---
        if active_command:
            # El candado absoluto. Da igual el modo, el robot nunca excederá estos límites.
            target.x = np.clip(target.x, -self.safe_x_limit, self.safe_x_limit)
            target.y = np.clip(target.y, -self.safe_y_limit, self.safe_y_limit)
            target.z = np.clip(target.z, self.safe_z_min, self.safe_z_max)
            
            # Actualizamos el objetivo virtual en TELEOP por si chocó contra el muro virtual
            if self.state == State.TELEOP:
                self.teleop_target = np.array([target.x, target.y, target.z])
                
            self.target_pub.publish(target)

def main():
    rclpy.init()
    node = TrajectoryPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()