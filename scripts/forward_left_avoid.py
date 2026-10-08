import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ForwardLeftAvoid(Node):

    def __init__(self):
        super().__init__('forward_left_avoid')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.front = float('inf')
        self.scan_received = False

        self.turning = False
        self.turn_start = 0.0

        self.timer = self.create_timer(
            0.05,
            self.control
        )

        self.get_logger().info(
            '前进 + 遇障碍左转程序启动'
        )

    def get_front_distance(self, msg):

        values = []

        for i, r in enumerate(msg.ranges):

            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            # 只看机器人正前方 ±20°
            if abs(angle) <= math.radians(20):

                if math.isfinite(r):

                    if (
                        msg.range_min
                        < r
                        < msg.range_max
                    ):
                        values.append(r)

        if not values:
            return float('inf')

        return min(values)

    def scan_callback(self, msg):

        self.front = self.get_front_distance(msg)
        self.scan_received = True

    def control(self):

        cmd = Twist()

        # 还没有收到激光数据
        if not self.scan_received:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)
            return

        # =========================
        # 正在左转
        # =========================

        if self.turning:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.8

            self.cmd_pub.publish(cmd)

            # 左转约 90°
            if (
                self.get_clock().now().nanoseconds / 1e9
                - self.turn_start
                > 1.9
            ):

                self.turning = False

            return

        # =========================
        # 前方障碍物
        # =========================

        if self.front < 0.60:

            # 停止前进
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            # 开始左转
            self.turning = True

            self.turn_start = (
                self.get_clock().now().nanoseconds / 1e9
            )

            return

        # =========================
        # 前方安全
        # =========================

        cmd.linear.x = 0.25
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def stop(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        for _ in range(10):
            self.cmd_pub.publish(cmd)


def main():

    rclpy.init()

    node = ForwardLeftAvoid()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
