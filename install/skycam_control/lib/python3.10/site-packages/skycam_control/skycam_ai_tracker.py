#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data

class AITracker(Node):
    def __init__(self):
        super().__init__('skycam_ai_tracker')
        
        # Suscripción instantánea (Anti-Lag)
        self.subscription = self.create_subscription(
            Image,
            '/skycam/main_camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data) 
            
        self.cmd_pub = self.create_publisher(Twist, '/skycam/cmd_vel', 10)
        self.bridge = CvBridge()
        
        self.get_logger().info("Cargando modelo YOLOv8n...")
        self.model = YOLO('yolov8n.pt') 
        self.get_logger().info("IA Tracker: Filtro de Masa Activo. Detección garantizada.")

        self.kp_x = 0.065  
        self.kp_y = 0.065  

        # --- NUEVOS REFLEJOS (Derivada Visual) ---
        # Estos parámetros detectan los "tirones" bruscos
        self.kd_x = 0.015  
        self.kd_y = 0.015  
        
        # Memoria para calcular a qué velocidad se mueve el error
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.deadband = 30  

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error procesando imagen: {e}")
            return

        img_h, img_w, _ = cv_image.shape
        center_x = img_w // 2
        center_y = img_h // 2

        # 1. YOLO AL MÁXIMO RENDIMIENTO
        # Subimos la resolución a 640 para que vea la pelota desde la estratosfera
        results = self.model.predict(
            source=cv_image, 
            classes=[32, 0], 
            verbose=False, 
            device='cpu', 
            imgsz=640,
            conf=0.10
        )
        
        cmd = Twist()
        target_found = False
        obj_x, obj_y = 0, 0

        if len(results[0].boxes) > 0:
            box = results[0].boxes[0].xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = map(int, box)
            obj_x = (x1 + x2) // 2
            obj_y = (y1 + y2) // 2
            
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(cv_image, f"YOLO ({results[0].boxes[0].conf[0]:.2f})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            target_found = True

        else:
            # 2. EL FILTRO DE MASA (OPENCV)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Umbral súper permisivo (130) para captar la pelota gris oscura
            _, thresh = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY)
            
            # Mostramos la máscara para depurar
            cv2.imshow("Debug Thresh (Ver Mascara)", cv2.resize(thresh, (320, 240)))
            
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # LA MAGIA: Filtramos todo lo que sea polvo (<3 px) o sombras del estadio (>500 px)
                valid_contours = [c for c in contours if 3 < cv2.contourArea(c) < 500]
                
                if valid_contours:
                    # De los contornos válidos (tamaño pelota), cogemos el más grande
                    largest_contour = max(valid_contours, key=cv2.contourArea)
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        obj_x = int(M["m10"] / M["m00"])
                        obj_y = int(M["m01"] / M["m00"])
                        cv2.circle(cv_image, (obj_x, obj_y), int(np.sqrt(cv2.contourArea(largest_contour)/np.pi)), (255, 0, 0), 2)
                        cv2.putText(cv_image, "OpenCV Tracker", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                        target_found = True

        # 3. COMANDOS DE VELOCIDAD
        if target_found:
            cv2.circle(cv_image, (obj_x, obj_y), 5, (0, 0, 255), -1)
            cv2.line(cv_image, (center_x, center_y), (obj_x, obj_y), (0, 0, 255), 2)
            
            # 1. Calculamos el error bruto actual
            error_x_raw = obj_x - center_x  
            error_y_raw = obj_y - center_y  
            
            # 2. Calculamos la Tasa de Cambio (Derivada = Regate brusco)
            d_error_x = error_x_raw - self.prev_error_x
            d_error_y = error_y_raw - self.prev_error_y
            
            # 3. Guardamos el error para el siguiente fotograma
            self.prev_error_x = error_x_raw
            self.prev_error_y = error_y_raw
            
            # 4. Aplicamos la Zona Muerta solo a la posición, no a los reflejos
            error_x_p = 0 if abs(error_x_raw) < self.deadband else error_x_raw
            error_y_p = 0 if abs(error_y_raw) < self.deadband else error_y_raw
            
            # Subimos un poco la velocidad punta permitida
            max_speed = 7 
            
            # LA FÓRMULA PD: Reacciona a la posición (Kp) + Anticipa el movimiento (Kd)
            vel_y_deseada = float(- (error_y_p * self.kp_y) - (d_error_y * self.kd_y))
            vel_x_deseada = float(- (error_x_p * self.kp_x) - (d_error_x * self.kd_x))
            
            cmd.linear.x = float(np.clip(vel_y_deseada, -max_speed, max_speed))
            cmd.linear.y = float(np.clip(vel_x_deseada, -max_speed, max_speed))
            cmd.linear.z = 0.0 
            
            self.cmd_pub.publish(cmd)
        else:
            # Si perdemos el objetivo, borramos la memoria para no dar tirones falsos al recuperarlo
            self.prev_error_x = 0.0
            self.prev_error_y = 0.0
            self.cmd_pub.publish(Twist())
            
        cv2.imshow("Skycam AI Vision", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = AITracker()
    rclpy.spin(node)
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()