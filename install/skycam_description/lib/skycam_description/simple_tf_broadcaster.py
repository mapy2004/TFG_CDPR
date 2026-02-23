#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class SkycamBroadcaster(Node):
    def __init__(self):
        super().__init__('skycam_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.broadcast_timer_callback) # 20 Hz
        self.t = 0.0

    def broadcast_timer_callback(self):
        t = TransformStamped()

        # Cabecera
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'platform'

        # Movimiento simulado (Un circulo suave para ver que vive)
        # En el futuro, aquí leerás la posición real calculada por tus cables
        t.transform.translation.x = 11.0 + 2.0 * math.sin(self.t)
        t.transform.translation.y = 7.5 + 2.0 * math.cos(self.t)
        t.transform.translation.z = 5.0

        # Orientación fija (plana) por ahora
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publicar
        self.tf_broadcaster.sendTransform(t)
        self.t += 0.05

def main():
    rclpy.init()
    node = SkycamBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()