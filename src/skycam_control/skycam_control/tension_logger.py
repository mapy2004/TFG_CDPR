#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
import csv
import time
import numpy as np
import os

class TensionLogger(Node):
    def __init__(self):
        super().__init__('skycam_tension_logger')
        
        # Archivo de salida
        self.filename = 'tensiones_vuelo.csv'
        
        # Inicializamos el CSV con las cabeceras
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Tiempo_s', 'Tension_1_N', 'Tension_2_N', 'Tension_3_N', 'Tension_4_N'])
            
        # Memoria para el muestreo síncrono
        self.tensions = [0.0, 0.0, 0.0, 0.0]
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Suscriptores a los tópicos de tu controlador
        self.create_subscription(Wrench, '/skycam/cmd_force_1', self.cb_t1, 10)
        self.create_subscription(Wrench, '/skycam/cmd_force_2', self.cb_t2, 10)
        self.create_subscription(Wrench, '/skycam/cmd_force_3', self.cb_t3, 10)
        self.create_subscription(Wrench, '/skycam/cmd_force_4', self.cb_t4, 10)
        
        # Timer de grabación a 20 Hz (cada 0.05 segundos)
        self.timer = self.create_timer(0.05, self.log_data)
        
        self.get_logger().info(f'Logger iniciado. Grabando datos en [{os.path.abspath(self.filename)}] a 20 Hz...')
        self.get_logger().info('Haz volar el dron y pulsa Ctrl+C aquí cuando termines.')

    # Callbacks: Calculamos el módulo (norma) del vector de fuerza para obtener la tensión escalar
    def cb_t1(self, msg):
        self.tensions[0] = np.linalg.norm([msg.force.x, msg.force.y, msg.force.z])
        
    def cb_t2(self, msg):
        self.tensions[1] = np.linalg.norm([msg.force.x, msg.force.y, msg.force.z])
        
    def cb_t3(self, msg):
        self.tensions[2] = np.linalg.norm([msg.force.x, msg.force.y, msg.force.z])
        
    def cb_t4(self, msg):
        self.tensions[3] = np.linalg.norm([msg.force.x, msg.force.y, msg.force.z])
        
    def log_data(self):
        # Tiempo relativo desde que se inició el script
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        # Guardado en disco
        with open(self.filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                f"{current_time:.3f}", 
                f"{self.tensions[0]:.2f}", 
                f"{self.tensions[1]:.2f}", 
                f"{self.tensions[2]:.2f}", 
                f"{self.tensions[3]:.2f}"
            ])

def main():
    rclpy.init()
    node = TensionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nGrabación detenida con éxito. CSV guardado.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()