#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf_transformations as tft


class ImuSwapNode(Node):

    def __init__(self):
        super().__init__('imu_swap_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            Imu,
            '/sensing/gnss/robins/ros/imu',
            self.imu_callback,
            qos
        )

        self.pub = self.create_publisher(
            Imu,
            '/imu_swap',
            qos
        )

        self.get_logger().info('IMU swap node started')


    def imu_callback(self, msg: Imu):
        q = msg.orientation

        # Quaternion -> RPY
        roll, pitch, yaw = tft.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

        # 🔁 Roll ↔ Pitch
        roll_new = pitch
        pitch_new = roll
        yaw_new = -yaw   # aynen kalsın

        # RPY -> Quaternion
        q_new = tft.quaternion_from_euler(
            roll_new,
            pitch_new,
            yaw_new
        )

        # Yeni IMU mesajı
        out = Imu()
        out.header = msg.header
        out.header.frame_id = msg.header.frame_id

        out.orientation.x = q_new[0]
        out.orientation.y = q_new[1]
        out.orientation.z = q_new[2]
        out.orientation.w = q_new[3]

        # Diğer alanları aynen geçir
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuSwapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
