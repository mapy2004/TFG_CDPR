#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class SkycamHighLevelController(Node):

    def __init__(self):
        super().__init__('skycam_controller')

        # Publisher de referencia
        self.ref_pub = self.create_publisher(
            Point,
            '/skycam/position_reference',
            10
        )

        # >>> AÑADIDO: subscriber de goal <<<
        self.goal_sub = self.create_subscription(
            Point,
            '/skycam/goal',
            self.goal_callback,
            10
        )

        # Estado interno
        self.current_ref = np.array([11.0, 7.5, 6.0])
        self.target = self.current_ref.copy()

        self.goal_active = False  # <<< AÑADIDO

        # Parámetros de planificación
        self.max_speed = 0.5  # m/s
        self.dt = 0.05

        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('Skycam high-level controller started')

    # >>> AÑADIDO <<<
    def goal_callback(self, msg):
        self.set_target(msg.x, msg.y, msg.z)
        self.goal_active = True

    def set_target(self, x, y, z):
        self.target = np.array([x, y, z])
        self.get_logger().info(f'New target: {self.target}')

    def update(self):
        error = self.target - self.current_ref

        # Umbral por eje
        if (
            abs(error[0]) < 0.01 and
            abs(error[1]) < 0.01 and
            abs(error[2]) < 0.01
        ):
            # >>> AÑADIDO: publicar una última vez <<<
            if self.goal_active:
                msg = Point()
                msg.x, msg.y, msg.z = self.current_ref.tolist()
                self.ref_pub.publish(msg)
                self.goal_active = False
            return

        # Velocidad deseada por eje
        v_des = np.clip(
            error / self.dt,
            -self.max_speed,
            self.max_speed
        )

        # Integración
        self.current_ref += v_des * self.dt

        msg = Point()
        msg.x, msg.y, msg.z = self.current_ref.tolist()
        self.ref_pub.publish(msg)


def main():
    rclpy.init()
    node = SkycamHighLevelController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
