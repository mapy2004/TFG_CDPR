#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class SkycamGoalSender(Node):

    def __init__(self, x, y, z):
        super().__init__('skycam_goal_sender')

        self.pub = self.create_publisher(
            Point,
            '/skycam/position_reference',
            10
        )

        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = z

        self.pub.publish(msg)
        self.get_logger().info(
            f'Objetivo enviado: x={x}, y={y}, z={z}'
        )


def main():
    rclpy.init()

    if len(sys.argv) != 4:
        print('Uso: ros2 run skycam_control skycam_goal_sender x y z')
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])

    node = SkycamGoalSender(x, y, z)

    # 🔹 dejamos respirar al middleware
    rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
