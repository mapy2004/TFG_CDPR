#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Wrench, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker 
from std_msgs.msg import ColorRGBA
from scipy.spatial.transform import Rotation as R
from scipy.optimize import nnls

class SkycamPaperControl(Node):

    def __init__(self):
        super().__init__('skycam_paper_control')

        # === FÍSICA ===
        self.mass = 25.0
        self.g = 9.81
        
        self.anchors = np.array([
            [ 10.0,  10.0, 15.0], [ 10.0, -10.0, 15.0],
            [-10.0,  10.0, 15.0], [-10.0, -10.0, 15.0]
        ])

        val = 0.45
        self.body_points = np.array([
            [ val,  val, 0.05], [ val, -val, 0.05], 
            [-val,  val, 0.05], [-val, -val, 0.05]  
        ])

        self.com_offset_body = np.array([0.0, 0.0, -0.2])

        # === ESTADO ===
        self.target_pos = np.array([0.0, 0.0, 6.0]) 
        self.smooth_ref = np.array([0.0, 0.0, 2.0]) 
        self.current_pos = None
        self.current_rot = np.eye(3)
        
        self.vel_lin_filtered = np.zeros(3)
        self.vel_ang_filtered = np.zeros(3)
        self.alpha = 0.4 

        # === PID ===
        self.kp_pos = np.array([  20.0,   20.0,  150.0]) 
        self.kd_pos = np.array([  15.0,   15.0,   60.0]) # Un poco más de freno lineal
        
        self.kp_yaw = 15.0  
        self.kd_yaw = 5.0
        
        # Damping activo para Pitch y Roll (evita que pendulee)
        self.kd_tilt = 30.0 
        
        # Pesos para el solver (Prioridad absoluta a no volcar)
        self.weight_pos = 1.0
        self.weight_rot = 15.0 

        self.min_tension = 15.0 
        self.max_tension = 4000.0
        self.dt = 0.01 
        
        self.create_subscription(Odometry, '/skycam/platform_odom', self.odom_cb, 10)
        self.pub_1 = self.create_publisher(Wrench, '/skycam/cmd_force_1', 10)
        self.pub_2 = self.create_publisher(Wrench, '/skycam/cmd_force_2', 10)
        self.pub_3 = self.create_publisher(Wrench, '/skycam/cmd_force_3', 10)
        self.pub_4 = self.create_publisher(Wrench, '/skycam/cmd_force_4', 10)
        self.marker_pub = self.create_publisher(Marker, '/skycam/cable_visuals', 10)
        
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info('SKYCAM: Controlador 6-DOF Ponderado (Weighted NNLS) Activado.')

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        if np.isnan(p.x): return
        
        self.current_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        self.current_rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        v_lin = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        v_ang = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        
        self.vel_lin_filtered = (self.alpha * v_lin) + ((1 - self.alpha) * self.vel_lin_filtered)
        self.vel_ang_filtered = (self.alpha * v_ang) + ((1 - self.alpha) * self.vel_ang_filtered)
        
        if np.linalg.norm(self.smooth_ref - np.array([0.0,0.0,2.0])) < 0.1:
             self.smooth_ref = self.current_pos.copy()

    def update(self):
        if self.current_pos is None: return

        alpha_ref = 0.02
        self.smooth_ref = self.smooth_ref * (1 - alpha_ref) + self.target_pos * alpha_ref

        # --- 1. DEMANDAS DE FUERZA (XYZ) ---
        err_pos = self.smooth_ref - self.current_pos
        f_pid = (self.kp_pos * err_pos) + (self.kd_pos * -self.vel_lin_filtered)
        
        max_lat = 25.0
        f_pid[0] = np.clip(f_pid[0], -max_lat, max_lat)
        f_pid[1] = np.clip(f_pid[1], -max_lat, max_lat)
        
        f_gravity = np.array([0.0, 0.0, self.mass * self.g])
        f_desired = f_pid + f_gravity

        # --- 2. DEMANDAS DE MOMENTO (Roll, Pitch, Yaw) ---
        # Damping activo: Si la plataforma se inclina, pedimos un torque contrario para frenarla
        M_x = -self.kd_tilt * self.vel_ang_filtered[0]
        M_y = -self.kd_tilt * self.vel_ang_filtered[1]

        r_err_vec = (R.identity() * R.from_matrix(self.current_rot).inv()).as_rotvec()
        err_yaw = r_err_vec[2] 
        M_z = (self.kp_yaw * err_yaw) + (self.kd_yaw * -self.vel_ang_filtered[2])

        # Limitamos los torques máximos demandados
        M_x = np.clip(M_x, -40.0, 40.0)
        M_y = np.clip(M_y, -40.0, 40.0)
        M_z = np.clip(M_z, -20.0, 20.0)

        # Wrench 6-DOF completo
        W_desired = np.array([f_desired[0], f_desired[1], f_desired[2], M_x, M_y, M_z])

        # --- 3. JACOBIANA 6x4 COMPLETA ---
        J = np.zeros((6, 4)) 
        marker_points = []
        u_vectors = []
        
        for i in range(4):
            r_body = self.body_points[i] - self.com_offset_body
            r_world = self.current_rot @ r_body
            
            b_global = self.current_pos + (self.current_rot @ self.body_points[i])
            L = self.anchors[i] - b_global
            dist = np.linalg.norm(L)
            u = L / dist if dist > 0.01 else np.array([0,0,1])
            u_vectors.append(u)
            
            # Balance de fuerzas
            J[0:3, i] = u 
            # Balance de momentos (r x F)
            tau_i = np.cross(r_world, u)
            J[3:6, i] = tau_i 
            
            marker_points.append(Point(x=self.anchors[i][0], y=self.anchors[i][1], z=self.anchors[i][2]))
            marker_points.append(Point(x=b_global[0], y=b_global[1], z=b_global[2]))
        
        self.publish_cables(marker_points)

        # --- 4. APLICACIÓN DE PESOS (WEIGHTED LEAST SQUARES) ---
        # Multiplicamos la jacobiana y el Wrench por la matriz de pesos
        # Esto obliga al solver a priorizar la estabilización angular
        Weights = np.diag([self.weight_pos, self.weight_pos, self.weight_pos, 
                           self.weight_rot, self.weight_rot, self.weight_rot])
        
        J_weighted = Weights @ J
        W_weighted = Weights @ W_desired

        # --- 5. SOLVER NNLS CON PRETENSIÓN ---
        T_min_vec = np.full(4, self.min_tension)
        # Restamos del Wrench deseado la fuerza que ya generan esos 15 N base
        W_adjusted = W_weighted - (J_weighted @ T_min_vec)
        
        try:
            delta_tensions, _ = nnls(J_weighted, W_adjusted)
            tensions = delta_tensions + T_min_vec
        except Exception as e:
            self.get_logger().error(f"Error en NNLS: {e}")
            tensions = T_min_vec

        tensions = np.clip(tensions, self.min_tension, self.max_tension)

        # --- 6. PUBLICAR ---
        self.publish_force(self.pub_1, tensions[0], u_vectors[0])
        self.publish_force(self.pub_2, tensions[1], u_vectors[1])
        self.publish_force(self.pub_3, tensions[2], u_vectors[2])
        self.publish_force(self.pub_4, tensions[3], u_vectors[3])

    def publish_force(self, pub, scalar, vec):
        msg = Wrench()
        f = scalar * vec
        msg.force.x, msg.force.y, msg.force.z = float(f[0]), float(f[1]), float(f[2])
        pub.publish(msg)

    def publish_cables(self, points):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = "cables", 0, Marker.LINE_LIST, Marker.ADD
        m.scale.x, m.color.r, m.color.a = 0.04, 0.1, 1.0
        m.points = points
        self.marker_pub.publish(m)

def main():
    rclpy.init()
    node = SkycamPaperControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()