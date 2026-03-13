#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from enum import Enum
from scipy.spatial.transform import Rotation as R
import csv # <--- NUEVO: Librería para guardar CSV
import os

class State(Enum):
    IDLE = 0
    APPROACHING_START = 1
    TELEOP = 2  
    EMERGENCY = 3   
    
class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('skycam_trajectory_planner')
        
        self.target_pub = self.create_publisher(Point, '/skycam/target_pos', 10)
        self.odom_sub = self.create_subscription(Odometry, '/skycam/platform_odom', self.odom_cb, 10)
        self.cmd_sub = self.create_subscription(Twist, '/skycam/cmd_vel', self.cmd_cb, 10)
        
        self.current_pos = None
        self.current_yaw = 0.0 
        
        self.virtual_target = np.array([0.0, 0.0, 0.0])
        self.state = State.IDLE
        
        self.approach_tolerance = 0.15 
        
        self.dt = 0.02       
        self.Z_height = 15.0   # ALTURA DE ESTADIO (15 metros)
        
        self.cmd_vel = np.zeros(3)
        self.virtual_vel = np.zeros(3)
        
        self.max_speed = 15.0       
        
        self.prev_cmd_angle = 0.0
        self.smooth_omega = 0.0 
        
        self.safe_x_limit = 8.0  
        self.safe_y_limit = 8.0  
        self.safe_z_max = 20.0   # Techo subido a 20 metros

        # --- NUEVO: PREPARACIÓN DEL ARCHIVO CSV ---
        self.csv_filename = 'trajectory_log.csv'
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Escribimos la cabecera del CSV
        self.csv_writer.writerow(['time', 'target_x', 'target_y', 'target_z', 'actual_x', 'actual_y', 'actual_z'])
        self.start_time = None
        # ------------------------------------------

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('Planificador: Modo Estadio (15m) ACTIVADO. Guardando log en CSV...')

    def evaluate_path(self, s):
        return np.array([0.0, 0.0, self.Z_height])

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        if not np.isnan(p.x):
            self.current_pos = np.array([p.x, p.y, p.z])
            q = msg.pose.pose.orientation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            self.current_yaw = r.as_euler('xyz', degrees=False)[2]
            
            if self.state == State.IDLE:
                self.virtual_target = self.current_pos.copy()
                self.state = State.APPROACHING_START

    def cmd_cb(self, msg):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])

    def control_loop(self):
        if self.current_pos is None: return
        
        # Inicializar el tiempo en el primer bucle
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9

        target = Point()
        active_command = False

        if self.state in [State.APPROACHING_START, State.TELEOP]:
            error_critico = np.linalg.norm(self.virtual_target - self.current_pos)
            # Si el robot físico se desvía más de 3.5 metros de su objetivo virtual, algo grave ha roto
            if error_critico > 3.5:
                self.state = State.EMERGENCY
                self.get_logger().error(f'¡FALLO CRÍTICO DE CONTROL! Desviación de {error_critico:.2f}m. Activando Emergencia.')

        if self.state == State.APPROACHING_START:
            start_point = self.evaluate_path(0.0)
            dir_vec = start_point - self.virtual_target
            dist = np.linalg.norm(dir_vec)
            step = 1.5 * self.dt 
            
            if dist > step:
                self.virtual_target += (dir_vec / dist) * step
            else:
                self.virtual_target = start_point
                
            target.x, target.y, target.z = self.virtual_target[0], self.virtual_target[1], self.virtual_target[2]
            active_command = True
            
            error_fisico = np.linalg.norm(start_point - self.current_pos)
            if dist <= step and error_fisico < self.approach_tolerance:
                self.state = State.TELEOP
                self.get_logger().info('MFS: ¡IA AL MANDO!')

        elif self.state == State.TELEOP:
            yaw = self.current_yaw
            v_x_global = self.cmd_vel[0] * np.cos(yaw) - self.cmd_vel[1] * np.sin(yaw)
            v_y_global = self.cmd_vel[0] * np.sin(yaw) + self.cmd_vel[1] * np.cos(yaw)
            cmd_vel_global = np.array([v_x_global, v_y_global, 0.0])

            vel_norm = np.linalg.norm(self.virtual_vel)
            cmd_norm = np.linalg.norm(cmd_vel_global)
            
            # --- 1. DETECCIÓN REAL DE CURVA (Anti-Órbita) ---
            if cmd_norm > 0.5:
                current_cmd_angle = np.arctan2(cmd_vel_global[1], cmd_vel_global[0])
                delta_angle = (current_cmd_angle - self.prev_cmd_angle + np.pi) % (2*np.pi) - np.pi
                self.prev_cmd_angle = current_cmd_angle
                
                if abs(delta_angle) > 1.5:
                    self.smooth_omega = 0.0  
                else:
                    omega_raw = delta_angle / self.dt
                    self.smooth_omega = (self.smooth_omega * 0.75) + (omega_raw * 0.25) #-- aquí metemos un filtro de media movil exponencial --#
            else:
                self.smooth_omega = self.smooth_omega * 0.50

            # --- 2. FRENADA CINEMATOGRÁFICA Y LOOKAHEAD ELÁSTICO ---
            current_lookahead = np.clip(cmd_norm * 0.08, 0.05, 0.50)
            if cmd_norm < vel_norm - 0.1:
                aceleracion_dinamica = 15.0  
            else:
                aceleracion_dinamica = np.clip(cmd_norm * 3.5, 3.0, 15.0)

            # Integramos la velocidad
            vel_error = cmd_vel_global - self.virtual_vel
            accel_cmd = vel_error / self.dt
            accel_norm = np.linalg.norm(accel_cmd)
            
            if aceleracion_dinamica > 0.0 and accel_norm > aceleracion_dinamica:
                accel_cmd = (accel_cmd / accel_norm) * aceleracion_dinamica
            
            self.virtual_vel += accel_cmd * self.dt
            
            vel_norm = np.linalg.norm(self.virtual_vel)
            if vel_norm > self.max_speed:
                self.virtual_vel = (self.virtual_vel / vel_norm) * self.max_speed

            # --- 3. EL LOOKAHEAD CURVO ---
            theta_pred = self.smooth_omega * current_lookahead
            cos_theta = np.cos(theta_pred)
            sin_theta = np.sin(theta_pred)
            
            v_x_curvo = self.virtual_vel[0] * cos_theta - self.virtual_vel[1] * sin_theta
            v_y_curvo = self.virtual_vel[0] * sin_theta + self.virtual_vel[1] * cos_theta

            target.x = self.current_pos[0] + (v_x_curvo * current_lookahead)
            target.y = self.current_pos[1] + (v_y_curvo * current_lookahead)
            target.z = self.Z_height
            active_command = True

        elif self.state == State.EMERGENCY:
            # 1. Matamos cualquier inercia o comando de la IA
            self.virtual_vel = np.zeros(3)
            
            # 2. Clavamos la posición XY actual y bajamos Z lentamente (0.5 m/s)
            target.x = self.current_pos[0]
            target.y = self.current_pos[1]
            target.z = max(0.0, self.current_pos[2] - (0.5 * self.dt))
            
            active_command = True
            self.get_logger().info('¡EMERGENCIA! Abortando misión y descendiendo...', throttle_duration_sec=1.0)

        if active_command:
            target.x = np.clip(target.x, -self.safe_x_limit, self.safe_x_limit)
            target.y = np.clip(target.y, -self.safe_y_limit, self.safe_y_limit)
            target.z = np.clip(target.z, 0.0, self.safe_z_max) 
            self.target_pub.publish(target)

            # --- NUEVO: GUARDAR DATOS EN EL CSV EN CADA BUCLE ---
            # Solo guardamos a partir de que entramos en TELEOP para no ensuciar la gráfica con el despegue
            if self.state == State.TELEOP:
                current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
                self.csv_writer.writerow([
                    current_time,
                    target.x, target.y, target.z,
                    self.current_pos[0], self.current_pos[1], self.current_pos[2]
                ])
                self.csv_file.flush() # Fuerza a guardar en el disco inmediatamente
        

def main():
    rclpy.init()
    node = TrajectoryPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close() # Cierra el archivo al hacer Ctrl+C
        rclpy.shutdown()

if __name__ == '__main__':
    main()
