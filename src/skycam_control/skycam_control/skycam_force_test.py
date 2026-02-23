#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from gazebo_msgs.srv import ApplyLinkWrench


class SkycamForceTest(Node):

    def __init__(self):
        super().__init__('skycam_force_test')

        self.cli = self.create_client(
            ApplyLinkWrench,
            '/apply_link_wrench'
        )

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo...')

        self.timer = self.create_timer(0.1, self.apply_force)

    def apply_force(self):
        req = ApplyLinkWrench.Request()
        req.link_name = 'platform'
        req.reference_frame = 'world'

        req.wrench.force.z = 6000.0  # fuerza hacia arriba
        req.duration.sec = 0
        req.duration.nanosec = 0

        self.cli.call_async(req)


def main():
    rclpy.init()
    node = SkycamForceTest()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
