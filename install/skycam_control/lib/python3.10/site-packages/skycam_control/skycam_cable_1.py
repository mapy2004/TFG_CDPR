#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry

class SkycamCable1(Node):

    def __init__(self):
        super().__init__('skycam_cable_1')

        # =========================
        # PARÁMETROS FÍSICOS (HEAVY DUTY)
        # =========================
        self.mass = 570.0  # Masa real
        self.g = 9.81
        self.anchor = np.array([0.0, 0.0, 10.0])

        # Rampa de referencia
        self.z_target = 6.0        
        self.z_ref_current = 0.5   
        self.ramp_speed = 0.5      

        # =========================
        # GANANCIAS PID (RECALIBRADAS PARA 570 KG)
        # =========================
        # Regla general: Kp ~ Masa * 30, Kd ~ sqrt(Kp*Masa)
        self.kp = 15000.0   # Fuerza bruta para mover 570kg
        self.ki = 500.0     # Integral para eliminar error estático
        self.kd = 4000.0    # Amortiguación fuerte para evitar oscilaciones

        self.integral_error = 0.0 
        self.max_integral = 5000.0 # Más margen para la integral
        
        # Límite de fuerza:
        # Peso base = 5600 N. Necesitamos margen para acelerar.
        # 20.000 N permite aceleraciones de hasta 3g (aprox).
        self.max_force = 20000.0 

        self.platform_pos = None
        self.platform_vel = np.zeros(3)

        # =========================
        # COMUNICACIÓN
        # =========================
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/skycam/platform_odom', self.odom_cb, qos)
        self.force_pub = self.create_publisher(Wrench, '/skycam/cmd_force', 10)

        self.dt = 0.02 
        self.timer = self.create_timer(self.dt, self.update)
        
        self.get_logger().info('SKYCAM CABLE 1: Configuración PESADA (570kg) Lista.')

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.platform_pos = np.array([p.x, p.y, p.z])
        self.platform_vel = np.array([v.x, v.y, v.z])

    def update(self):
        if self.platform_pos is None:
            return

        # 1. Rampa
        if self.z_ref_current < self.z_target:
            self.z_ref_current += self.ramp_speed * self.dt
            if self.z_ref_current > self.z_target: 
                self.z_ref_current = self.z_target

        # 2. Vector Unitario
        u_vec = self.anchor - self.platform_pos
        dist = np.linalg.norm(u_vec)
        if dist < 1e-3: u_hat = np.array([0.0, 0.0, 1.0])
        else: u_hat = u_vec / dist 

        # 3. PID
        z_error = self.z_ref_current - self.platform_pos[2]
        vel_z = self.platform_vel[2]

        if abs(z_error) < 1.0:
            self.integral_error += z_error * self.dt
            self.integral_error = np.clip(self.integral_error, -self.max_integral, self.max_integral)
        else:
            self.integral_error = 0.0

        # FeedForward: Debe compensar 5591 Newtons de gravedad
        term_ff = self.mass * self.g 
        term_p  = self.kp * z_error
        term_i  = self.ki * self.integral_error
        term_d  = self.kd * vel_z

        tension_raw = term_ff + term_p + term_i - term_d
        
        # Saturación (0 a 20kN)
        tension = max(0.0, min(tension_raw, self.max_force))
        
        force_vector = tension * u_hat

        # 4. Publicar
        msg = Wrench()
        msg.force.x = float(force_vector[0])
        msg.force.y = float(force_vector[1])
        msg.force.z = float(force_vector[2])
        # Torque cero (tiramos del centro de masa)
        msg.torque.x = 0.0; msg.torque.y = 0.0; msg.torque.z = 0.0

        self.force_pub.publish(msg)

        self.get_logger().info(
            f"Ref:{self.z_ref_current:.2f} | Z:{self.platform_pos[2]:.2f} | T:{tension:.0f}N", 
            throttle_duration_sec=0.5
        )

def main():
    rclpy.init()
    node = SkycamCable1()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()