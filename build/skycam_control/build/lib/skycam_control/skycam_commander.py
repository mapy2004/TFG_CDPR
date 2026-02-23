#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import threading
import sys
import time

class SkycamCommander(Node):

    def __init__(self):
        super().__init__('skycam_commander')
        
        # Publicador de órdenes al "músculo"
        self.pub_setpoint = self.create_publisher(Point, '/skycam/setpoint', 10)
        
        # Estado actual del planificador
        self.current_setpoint = np.array([0.0, 0.0, 6.0]) # Empezamos aquí
        self.is_moving = False
        
        # Timer de bucle de control (50 Hz)
        self.dt = 0.02
        self.create_timer(self.dt, self.control_loop)
        
        # Variables de trayectoria
        self.traj_start_pos = None
        self.traj_end_pos = None
        self.traj_duration = 0.0
        self.traj_time_elapsed = 0.0
        
        self.get_logger().info("SKYCAM COMMANDER: Listo. Escribe coordenadas.")
        
        # Hilo para inputs de usuario sin bloquear ROS
        input_thread = threading.Thread(target=self.user_interface_loop)
        input_thread.daemon = True
        input_thread.start()

    def control_loop(self):
        if self.is_moving:
            self.traj_time_elapsed += self.dt
            
            # ¿Hemos llegado?
            if self.traj_time_elapsed >= self.traj_duration:
                self.current_setpoint = self.traj_end_pos
                self.is_moving = False
                self.get_logger().info(f"Objetivo alcanzado: {self.current_setpoint}")
            else:
                # Calcular punto intermedio (Polinomio Quíntico)
                # s(t) va de 0 a 1 suavemente
                tau = self.traj_time_elapsed / self.traj_duration
                # Formula mágica de movimiento suave (Minimum Jerk Trajectory)
                scaling_factor = 10*(tau**3) - 15*(tau**4) + 6*(tau**5)
                
                # Interpolación
                self.current_setpoint = self.traj_start_pos + (self.traj_end_pos - self.traj_start_pos) * scaling_factor

        # Publicar setpoint actual (estemos moviéndonos o quietos)
        msg = Point()
        msg.x = float(self.current_setpoint[0])
        msg.y = float(self.current_setpoint[1])
        msg.z = float(self.current_setpoint[2])
        self.pub_setpoint.publish(msg)

    def go_to(self, x, y, z, duration_sec=5.0):
        if self.is_moving:
            self.get_logger().warn("¡Espera! Todavía estoy moviéndome.")
            return

        self.traj_start_pos = self.current_setpoint.copy()
        self.traj_end_pos = np.array([x, y, z])
        self.traj_duration = duration_sec
        self.traj_time_elapsed = 0.0
        self.is_moving = True
        self.get_logger().info(f"Iniciando viaje a [{x}, {y}, {z}] en {duration_sec}s...")

    def user_interface_loop(self):
        time.sleep(1) # Esperar a que todo cargue
        print("\n=== SKYCAM COMMANDER ===")
        print("Formato: x y z tiempo")
        print("Ejemplo: 5 5 8 4  -> Ir a x=5, y=5, z=8 en 4 segundos")
        print("Ejemplo: 0 0 6 3  -> Volver al centro en 3 segundos")
        
        while rclpy.ok():
            try:
                line = input(">> ")
                parts = line.split()
                if len(parts) == 4:
                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2])
                    t = float(parts[3])
                    
                    # Límites de seguridad básicos
                    if z < 1.0 or z > 14.0:
                        print("ERROR: Z peligrosa (Mantener entre 1 y 14m)")
                        continue
                        
                    self.go_to(x, y, z, t)
                else:
                    print("Formato inválido. Usa: x y z tiempo")
            except ValueError:
                print("Error: Introduce números.")
            except Exception as e:
                print(f"Error: {e}")

def main():
    rclpy.init()
    node = SkycamCommander()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()