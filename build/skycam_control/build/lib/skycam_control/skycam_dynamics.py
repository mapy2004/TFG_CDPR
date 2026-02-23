#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point as GeoPoint


class SkycamDynamics(Node):

    def __init__(self):
        super().__init__('skycam_dynamics')

        # =========================
        # TF
        # =========================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # =========================
        # Parámetros físicos
        # =========================
        self.mass = 5.0  # kg
        self.g = np.array([0.0, 0.0, -9.81])

        # =========================
        # Estado de la plataforma
        # =========================
        self.pos = np.array([11.0, 7.5, 5.0])
        self.vel = np.zeros(3)

        # =========================
        # Referencia de posición
        # =========================
        self.pos_ref = np.array([11.0, 7.5, 6.0])
        self.create_subscription(
            Point,
            '/skycam/position_reference',
            self.reference_callback,
            10
        )


        # =========================
        # Control PD (justificado)
        # ζ = 1 (críticamente amortiguado)
        # ωn = 1 rad/s
        # =========================
        self.Kp = self.mass * 1.0**2        # 570 N/m
        self.Kd = 2.0 * self.mass * 1.0     # 1140 N·s/m

        # =========================
        # Puntos de anclaje (no usados aún)
        # =========================
        self.platform_anchors = {
            1: np.array([ 0.45,  0.45, 0.05]),
            2: np.array([-0.45,  0.45, 0.05]),
            3: np.array([-0.45, -0.45, 0.05]),
            4: np.array([ 0.45, -0.45, 0.05]),
        }

        # =========================
        # Simulación
        # =========================
        self.dt = 0.01  # 100 Hz
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('Skycam dynamics with PD position control started')
        self.state_pub = self.create_publisher(Point, '/skycam/position', 10)

        self.cable_pub = self.create_publisher(
            Marker,
            '/skycam/cables',
            10
        )


    def update(self):
        try:
            # =========================
            # Control PD por posición
            # =========================
            e_pos = self.pos_ref - self.pos
            e_vel = -self.vel

            F_ctrl = self.Kp * e_pos + self.Kd * e_vel

            # =========================
            # Fuerza total
            # =========================
            F_total = F_ctrl + self.mass * self.g

            # =========================
            # Dinámica
            # =========================
            acc = F_total / self.mass
            self.vel += acc * self.dt
            self.pos += self.vel * self.dt

            # =========================
            # Publicar TF
            # =========================
            self.publish_platform_tf()
            self.publish_cables()


        except Exception as e:
            self.get_logger().warn(f'Dynamics error: {e}')
        
        state = Point()
        state.x, state.y, state.z = self.pos.tolist()
        self.state_pub.publish(state)


    def publish_platform_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'platform'

        t.transform.translation.x = float(self.pos[0])
        t.transform.translation.y = float(self.pos[1])
        t.transform.translation.z = float(self.pos[2])

        t.transform.rotation.w = 1.0  # sin rotación

        self.tf_broadcaster.sendTransform(t)

    def reference_callback(self, msg):
        self.pos_ref = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(
            f'Nueva referencia recibida: {self.pos_ref}'
        )

    def publish_cables(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'skycam_cables'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.02

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []

        # posiciones de los anclajes de la plataforma en mundo
        platform_points = []
        for p in self.platform_anchors.values():
            platform_points.append(self.pos + p)

        for i in range(1, 5):
            try:
                tf_anchor = self.tf_buffer.lookup_transform(
                    'world',
                    f'anchor_{i}',
                    rclpy.time.Time()
                )

                anchor = np.array([
                    tf_anchor.transform.translation.x,
                    tf_anchor.transform.translation.y,
                    tf_anchor.transform.translation.z
                ])

                # buscar el anclaje de plataforma más cercano
                distances = [np.linalg.norm(anchor - p) for p in platform_points]
                idx = int(np.argmin(distances))
                closest = platform_points[idx]

                p_anchor = GeoPoint()
                p_anchor.x, p_anchor.y, p_anchor.z = anchor.tolist()

                p_plat = GeoPoint()
                p_plat.x, p_plat.y, p_plat.z = closest.tolist()

                marker.points.append(p_anchor)
                marker.points.append(p_plat)

            except Exception:
                pass

        self.cable_pub.publish(marker)





def main():
    rclpy.init()
    node = SkycamDynamics()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
