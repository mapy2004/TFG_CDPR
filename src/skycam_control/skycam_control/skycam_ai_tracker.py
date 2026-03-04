#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import threading
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation as R

class AITracker(Node):
    def __init__(self):
        super().__init__('skycam_ai_tracker')
        
        # --- PERFIL ANTI-LAG ESTRICTO ---
        # Memoria de 1 solo mensaje. Descartamos el pasado sin piedad.
        qos_strict = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            Image, '/skycam/main_camera/image_raw', self.image_callback, qos_strict) 
        self.odom_sub = self.create_subscription(
            Odometry, '/skycam/platform_odom', self.odom_cb, qos_strict)
        
        self.cmd_pub = self.create_publisher(Twist, '/skycam/cmd_vel', 10)
        self.bridge = CvBridge()
        
        self.get_logger().info("Cargando modelo YOLOv8n...")
        self.model = YOLO('yolov8n.pt') 
        self.get_logger().info("IA Tracker: MODO ZERO-LAG (QoS Estricto) ACTIVADO.")

        self.kp_x = 0.085  
        self.kp_y = 0.085  
        self.kd_x = 0.095  
        self.kd_y = 0.095 
        
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        
        self.deadband = 15  
        
        self.roll = 0.0
        self.pitch = 0.0
        self.fov_px = 650.0 
        self.comp_sign_x = 1.0   
        self.comp_sign_y = -1.0  
        
        self.last_cmd = Twist()
        self.frames_lost = 0
        self.max_memory_frames = 60 

        self.smooth_roll = 0.0
        self.smooth_pitch = 0.0
        self.alpha_gimbal = 1.0 
        
        self.smooth_vx = 0.0
        self.smooth_vy = 0.0
        self.alpha_cmd = 0.92 
        
        self.prev_angle = 0.0
        self.momentum = 0.15 

        self.latest_cv_image = None
        self.image_lock = threading.Lock()

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])
        euler = r.as_euler('xyz', degrees=False)
        self.roll = euler[0]
        self.pitch = euler[1]

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.image_lock:
                self.latest_cv_image = cv_img
        except Exception:
            pass

    def process_loop(self):
        with self.image_lock:
            if self.latest_cv_image is None:
                cv_image = None
            else:
                cv_image = self.latest_cv_image.copy()
                self.latest_cv_image = None

        if cv_image is None:
            cv2.waitKey(5)
            return

        display_image = cv_image.copy()

        img_h, img_w, _ = cv_image.shape
        center_x = img_w // 2
        center_y = img_h // 2

        self.smooth_roll = (self.smooth_roll * (1.0 - self.alpha_gimbal)) + (self.roll * self.alpha_gimbal)
        self.smooth_pitch = (self.smooth_pitch * (1.0 - self.alpha_gimbal)) + (self.pitch * self.alpha_gimbal)

        offset_x = int(self.smooth_roll * self.fov_px * self.comp_sign_x)
        offset_y = int(self.smooth_pitch * self.fov_px * self.comp_sign_y)
        
        gimbal_x = center_x + offset_x
        gimbal_y = center_y + offset_y

        cv2.drawMarker(display_image, (center_x, center_y), (0, 0, 150), cv2.MARKER_CROSS, 20, 1)
        cv2.drawMarker(display_image, (gimbal_x, gimbal_y), (0, 255, 0), cv2.MARKER_TILTED_CROSS, 15, 2)
        cv2.circle(display_image, (gimbal_x, gimbal_y), self.deadband, (255, 255, 0), 1)

        results = self.model.predict(source=cv_image, classes=[32, 0], verbose=False, device='cpu', imgsz=320, conf=0.10)
        
        cmd = Twist()
        target_found = False
        obj_x, obj_y = 0, 0

        if len(results[0].boxes) > 0:
            box = results[0].boxes[0].xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = map(int, box)
            obj_x = (x1 + x2) // 2
            obj_y = (y1 + y2) // 2
            cv2.rectangle(display_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            target_found = True
        else:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_orange = np.array([2, 120, 120])
            upper_orange = np.array([25, 255, 255])
            thresh = cv2.inRange(hsv, lower_orange, upper_orange)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                valid_contours = [c for c in contours if 1 < cv2.contourArea(c) < 500]
                if valid_contours:
                    largest_contour = max(valid_contours, key=cv2.contourArea)
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        obj_x = int(M["m10"] / M["m00"])
                        obj_y = int(M["m01"] / M["m00"])
                        target_found = True

        if target_found:
            error_x_raw = obj_x - gimbal_x  
            error_y_raw = obj_y - gimbal_y  
            error_dist_raw = np.hypot(error_x_raw, error_y_raw)
            
            if error_dist_raw <= self.deadband:
                error_x_p = 0.0
                error_y_p = 0.0
                error_dist = 0.0
            else:
                reduction_ratio = (error_dist_raw - self.deadband) / error_dist_raw
                error_x_p = float(error_x_raw * reduction_ratio)
                error_y_p = float(error_y_raw * reduction_ratio)
                error_dist = float(error_dist_raw - self.deadband)

            if self.frames_lost > 0:
                self.prev_error_x = error_x_p
                self.prev_error_y = error_y_p
                self.prev_angle = np.arctan2(error_y_p, error_x_p)
                self.frames_lost = 0 
                
            cv2.circle(display_image, (obj_x, obj_y), 5, (0, 0, 255), -1)
            cv2.line(display_image, (gimbal_x, gimbal_y), (obj_x, obj_y), (0, 255, 255), 2)
            
            # Filtro Anti-Amagos
            if error_dist > 5.0:
                current_angle = np.arctan2(error_y_p, error_x_p)
                angle_diff = abs((current_angle - self.prev_angle + np.pi) % (2*np.pi) - np.pi)
                
                if angle_diff > 0.78:
                    self.momentum = 0.15 
                else:
                    self.momentum = min(1.0, self.momentum + 0.05)
                    
                self.prev_angle = current_angle
            else:
                self.momentum = max(0.15, self.momentum - 0.05) 

            dist_ratio = np.clip(error_dist / 90.0, 0.0, 1.0)
            escala = np.clip(dist_ratio * self.momentum, 0.25, 1.0)
            
            curr_kp_x = self.kp_x * escala
            curr_kp_y = self.kp_y * escala
            curr_kd_x = self.kd_x * escala
            curr_kd_y = self.kd_y * escala
            
            d_error_x = error_x_p - self.prev_error_x
            d_error_y = error_y_p - self.prev_error_y
            
            self.prev_error_x = error_x_p
            self.prev_error_y = error_y_p
            
            max_speed_dinamico = np.clip(12.0 * escala, 4.0, 12.0) 
            
            vel_x_raw = float(- (error_x_p * curr_kp_x) - (d_error_x * curr_kd_x))
            vel_y_raw = float(- (error_y_p * curr_kp_y) - (d_error_y * curr_kd_y))
            
            self.smooth_vy = (self.smooth_vy * (1.0 - self.alpha_cmd)) + (vel_y_raw * self.alpha_cmd)
            self.smooth_vx = (self.smooth_vx * (1.0 - self.alpha_cmd)) + (vel_x_raw * self.alpha_cmd)
            
            cmd.linear.x = float(np.clip(self.smooth_vy, -max_speed_dinamico, max_speed_dinamico))
            cmd.linear.y = float(np.clip(self.smooth_vx, -max_speed_dinamico, max_speed_dinamico))
            cmd.linear.z = 0.0 
            
            self.last_cmd = cmd
            self.cmd_pub.publish(cmd)
            
        else:
            if self.frames_lost < self.max_memory_frames:
                self.frames_lost += 1
                self.last_cmd.linear.x *= 0.92 
                self.last_cmd.linear.y *= 0.92
                self.smooth_vx = self.last_cmd.linear.y
                self.smooth_vy = self.last_cmd.linear.x
                self.cmd_pub.publish(self.last_cmd)
                cv2.putText(display_image, "MEMORIA VISUAL...", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
            else:
                self.smooth_vx = 0.0
                self.smooth_vy = 0.0
                self.cmd_pub.publish(Twist())

        cv2.imshow("Skycam AI Vision", display_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = AITracker()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            node.process_loop()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()