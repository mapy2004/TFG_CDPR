#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Wrench, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker 
from scipy.spatial.transform import Rotation as R
from scipy.optimize import nnls

class SkycamPaperControl(Node):

    def __init__(self):
        super().__init__('skycam_paper_control')

        self.mass = 25.0
        self.g = 9.81
        
        self.anchors = np.array([
            [ 30.0,  20.0, 30.0], [ 30.0, -20.0, 30.0],
            [-30.0,  20.0, 30.0], [-30.0, -20.0, 30.0]
        ])

        val = 0.45
        self.body_points = np.array([
            [ val,  val, 0.05], [ val, -val, 0.05], 
            [-val,  val, 0.05], [-val, -val, 0.05]  
        ])

        self.com_offset_body = np.array([0.0, 0.0, -0.15])
        
        # --- AJUSTE VITAL: Altura inicial coherente con el planificador ---
        self.target_pos = np.array([0.0, 0.0, 15.0]) 
        
        self.current_pos = None
        self.current_rot = np.eye(3)
        self.vel_lin_filtered = np.zeros(3)
        self.vel_ang_filtered = np.zeros(3)
        
        self.alpha_lin = 0.6  
        self.alpha_ang = 0.85 
        
        self.kp_pos = np.array([ 185.0,  185.0,  200.0]) 
        self.kd_pos = np.array([ 145.0,  145.0,  125.0])
        self.kp_yaw = 200.0    
        self.kd_yaw = 100.0
        self.kp_tilt = 80.0   
        self.kd_tilt = 60.0
        
        self.min_tension = 10.0 
        self.max_tension = 800.0
        
        self.create_subscription(Point, '/skycam/target_pos', self.target_cb, 10)
        self.create_subscription(Odometry, '/skycam/platform_odom', self.odom_cb, 10)
        
        self.pub_1 = self.create_publisher(Wrench, '/skycam/cmd_force_1', 10)
        self.pub_2 = self.create_publisher(Wrench, '/skycam/cmd_force_2', 10)
        self.pub_3 = self.create_publisher(Wrench, '/skycam/cmd_force_3', 10)
        self.pub_4 = self.create_publisher(Wrench, '/skycam/cmd_force_4', 10)
        self.marker_pub = self.create_publisher(Marker, '/skycam/cable_visuals', 10)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        if np.isnan(p.x): return
        self.current_pos = np.array([p.x, p.y, p.z])
        q = msg.pose.pose.orientation
        self.current_rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        v_lin_local = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        v_ang_local = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

        v_lin_global = self.current_rot @ v_lin_local
        v_ang_global = self.current_rot @ v_ang_local

        self.vel_lin_filtered = (self.alpha_lin * v_lin_global) + ((1 - self.alpha_lin) * self.vel_lin_filtered)
        self.vel_ang_filtered = (self.alpha_ang * v_ang_global) + ((1 - self.alpha_ang) * self.vel_ang_filtered)
        self.update()

    def target_cb(self, msg):
        self.target_pos = np.array([msg.x, msg.y, msg.z])

    def update(self):
        if self.current_pos is None: return

        err_pos = self.target_pos - self.current_pos
        f_pid = (self.kp_pos * err_pos) + (self.kd_pos * -self.vel_lin_filtered) 
        
        max_lat = 250.0 
        f_pid[0] = np.clip(f_pid[0], -max_lat, max_lat)
        f_pid[1] = np.clip(f_pid[1], -max_lat, max_lat)
        f_pid[2] = np.clip(f_pid[2], -250.0, 250.0)
        
        f_gravity = np.array([0.0, 0.0, self.mass * self.g])
        f_desired = f_pid + f_gravity

        r_err_vec = (R.identity() * R.from_matrix(self.current_rot).inv()).as_rotvec()
        err_roll = r_err_vec[0]
        err_pitch = r_err_vec[1]
        err_yaw = r_err_vec[2] 

        M_x = (self.kp_tilt * err_roll) + (self.kd_tilt * -self.vel_ang_filtered[0])
        M_y = (self.kp_tilt * err_pitch) + (self.kd_tilt * -self.vel_ang_filtered[1])
        M_z = (self.kp_yaw * err_yaw) + (self.kd_yaw * -self.vel_ang_filtered[2])

        M_x = np.clip(M_x, -80.0, 80.0)
        M_y = np.clip(M_y, -80.0, 80.0)
        M_z = np.clip(M_z, -150.0, 150.0)

        W_desired = np.array([f_desired[0], f_desired[1], f_desired[2], M_x, M_y, M_z])

        J = np.zeros((6, 4)) 
        marker_points = []
        u_vectors = []
        L_xy_list = []  # NUEVO: Guardaremos la distancia horizontal de cada cable
        
        for i in range(4):
            r_body = self.body_points[i] - self.com_offset_body
            r_world = self.current_rot @ r_body
            
            b_global = self.current_pos + (self.current_rot @ self.body_points[i])
            L = self.anchors[i] - b_global
            dist = np.linalg.norm(L)
            
            # Calculamos la proyección horizontal del cable para la catenaria
            L_xy_list.append(np.linalg.norm(L[0:2]))
            
            u = L / dist if dist > 0.01 else np.array([0,0,1])
            u_vectors.append(u)
            
            J[0:3, i] = u 
            tau_i = np.cross(r_world, u)
            J[3:6, i] = tau_i 
            
            marker_points.append(Point(x=self.anchors[i][0], y=self.anchors[i][1], z=self.anchors[i][2]))
            marker_points.append(Point(x=b_global[0], y=b_global[1], z=b_global[2]))
        
        self.publish_cables(marker_points)

        Weights = np.diag([1.0, 1.0, 1.5, 5.0, 5.0, 10.0])
        
        J_weighted = Weights @ J
        W_weighted = Weights @ W_desired

        # --- RESTRICCIÓN DINÁMICA DE CATENARIA (SAGGING) ---
        rho = 0.05      # Densidad lineal del cable (50 gramos/metro)
        max_sag = 0.6   # Pandeo máximo permitido (60 cm)
        dynamic_t_min = []
        
        for i in range(4):
            # Fórmula de tensión mínima para evitar que el cable se combe más de max_sag
            t_sag = (rho * self.g * (L_xy_list[i]**2)) / (8.0 * max_sag)
            # Exigimos la mayor de las dos: o los 10N estructurales, o la fuerza anti-pandeo
            dynamic_t_min.append(max(self.min_tension, t_sag))
            
        T_min_vec = np.array(dynamic_t_min)
        # ----------------------------------------------------

        W_adjusted = W_weighted - (J_weighted @ T_min_vec)
        
        try:
            delta_tensions, _ = nnls(J_weighted, W_adjusted)
            tensions = delta_tensions + T_min_vec
        except Exception as e:
            tensions = T_min_vec

        tensions = np.clip(tensions, T_min_vec, self.max_tension)

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