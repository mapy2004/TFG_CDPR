#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class BallMover(Node):
    def __init__(self):
        super().__init__('ball_mover')
        
        # Publicador de comandos y Suscriptor para saber dónde está
        self.publisher_ = self.create_publisher(Twist, '/ball/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/ball/odom', self.odom_cb, 10)
        
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback) # 10Hz
        
        # Posición actual de la pelota
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Límite del WFW (Lo restringimos a 7 metros para tener margen de seguridad)
        self.wfw_limit = 7.0 
        
        # --- SECUENCIA DE ESTRÉS (Simulación de partido) ---
        # Formato: (Vel. Lineal, Vel. Angular, Duración en segundos)
        self.sequence = [
            (0.0,  0.0, 3.0),  # 1. PARADA (El jugador tiene el balón, prueba la estabilización)
            (3.0,  0.0, 2.0),  # 2. SPRINT (Pase largo al hueco)
            (0.0,  0.0, 1.5),  # 3. RECEPCIÓN (Frenada en seco)
            (1.5,  2.5, 2.0),  # 4. REGATE (Curva muy cerrada y rápida)
            (2.5, -0.8, 3.0),  # 5. CONDUCCIÓN (Curva amplia a alta velocidad)
            (-1.5, 0.0, 1.5),  # 6. PASE ATRÁS (Movimiento inverso repentino)
        ]
        self.current_step = 0
        self.time_in_step = 0.0

        self.get_logger().info('Generador de Rutas de Estrés con Geofence WFW Iniciado.')

    def odom_cb(self, msg):
        """ Actualiza la posición y orientación de la pelota """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extraer YAW del quaternion de Gazebo
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        msg = Twist()
        
        # --- 1. SEGURIDAD WFW (Geofence) ---
        dist_to_center = math.sqrt(self.x**2 + self.y**2)
        if dist_to_center > self.wfw_limit:
            # Si toca la pared virtual, forzamos un giro de regreso al centro (0,0)
            angle_to_center = math.atan2(-self.y, -self.x)
            angle_diff = angle_to_center - self.yaw
            
            # Normalizar el ángulo de -pi a pi
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            
            msg.linear.x = 1.5  # Velocidad de retorno
            # Girar hacia el centro
            msg.angular.z = 2.0 if angle_diff > 0 else -2.0 
            
            self.publisher_.publish(msg)
            self.get_logger().info('¡Pelota tocando límite WFW! Retornando al centro...', throttle_duration_sec=1.0)
            return

        # --- 2. EJECUCIÓN DE LA SECUENCIA DE ESTRÉS ---
        v, w, duration = self.sequence[self.current_step]
        
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        
        self.publisher_.publish(msg)
        
        # Avanzar el reloj de la secuencia
        self.time_in_step += self.dt
        if self.time_in_step >= duration:
            self.time_in_step = 0.0
            self.current_step = (self.current_step + 1) % len(self.sequence)
            self.get_logger().info(f'Cambiando a jugada {self.current_step + 1}/{len(self.sequence)}')

def main(args=None):
    rclpy.init(args=args)
    ball_mover = BallMover()
    rclpy.spin(ball_mover)
    ball_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()