import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs

class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')

        self.bridge = CvBridge()

        # =========================
        # RGB 图像
        # =========================
        self.image_topic = (
            '/world/bot2_complex_room/model/burger/'
            'link/camera_link/sensor/rgbd_camera/image'
        )

        # =========================
        # 深度图
        # =========================
        self.depth_topic = (
            '/world/bot2_complex_room/model/burger/'
            'link/camera_link/sensor/rgbd_camera/depth_image'
        )

        # =========================
        # 相机内参
        # =========================
        self.camera_info_topic = (
            '/world/bot2_complex_room/model/burger/'
            'link/camera_link/sensor/rgbd_camera/camera_info'
        )

        self.latest_depth = None
        self.camera_info = None

        # =========================
        # TF
        # =========================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =========================
        # RGB subscriber
        # =========================
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # =========================
        # Depth subscriber
        # =========================
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        # =========================
        # CameraInfo subscriber
        # =========================
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        # =========================
        # 检测结果图像
        # =========================
        self.image_pub = self.create_publisher(
            Image,
            '/vision/detected_image',
            10
        )

        self.get_logger().info(
            'Color + Depth + Map detector started.'
        )

    # =========================================================
    # 深度图回调
    # =========================================================
    def depth_callback(self, msg):

        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

        except Exception as e:
            self.get_logger().error(
                f'Depth conversion failed: {e}'
            )

    # =========================================================
    # 相机内参回调
    # =========================================================
    def camera_info_callback(self, msg):

        self.camera_info = msg

    # =========================================================
    # RGB 图像回调
    # =========================================================
    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:
            self.get_logger().error(
                f'Image conversion failed: {e}'
            )
            return

        # 如果还没有深度图或相机内参，先显示普通检测
        if self.latest_depth is None or self.camera_info is None:

            self.detect_colors(frame)

            output_msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding='bgr8'
            )

            output_msg.header = msg.header

            self.image_pub.publish(output_msg)

            return

        # =========================
        # HSV
        # =========================
        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        color_ranges = {

            'red': [
                (
                    (0, 100, 80),
                    (10, 255, 255)
                ),
                (
                    (170, 100, 80),
                    (179, 255, 255)
                )
            ],

            'green': [
                (
                    (35, 80, 50),
                    (85, 255, 255)
                )
            ],

            'blue': [
                (
                    (90, 80, 50),
                    (130, 255, 255)
                )
            ],

            'yellow': [
                (
                    (20, 80, 80),
                    (40, 255, 255)
                )
            ]
        }

        for color_name, ranges in color_ranges.items():

            mask = None

            for lower, upper in ranges:

                current_mask = cv2.inRange(
                    hsv,
                    lower,
                    upper
                )

                if mask is None:
                    mask = current_mask

                else:
                    mask = cv2.bitwise_or(
                        mask,
                        current_mask
                    )

            # =========================
            # 去噪
            # =========================
            kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT,
                (5, 5)
            )

            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel
            )

            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                kernel
            )

            # =========================
            # 查找轮廓
            # =========================
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:

                area = cv2.contourArea(contour)

                if area < 300:
                    continue

                x, y, w, h = cv2.boundingRect(
                    contour
                )

                # =========================
                # 目标中心像素
                # =========================
                u = x + w // 2
                v = y + h // 2

                # =========================
                # 获取深度
                # =========================
                depth = self.get_depth(
                    u,
                    v
                )

                # 画检测框
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (255, 255, 255),
                    2
                )

                # =========================
                # 默认标签
                # =========================
                label = (
                    f'{color_name} '
                    f'area={int(area)}'
                )

                # =========================
                # 如果有有效深度
                # =========================
                if depth is not None:

                    point_camera = (
                        self.pixel_to_3d(
                            u,
                            v,
                            depth
                        )
                    )

                    if point_camera is not None:

                        point_map = (
                            self.camera_to_map(
                                point_camera,
                                msg.header
                            )
                        )

                        if point_map is not None:

                            label = (
                                f'{color_name} '
                                f'X={point_map.point.x:.2f} '
                                f'Y={point_map.point.y:.2f} '
                                f'Z={point_map.point.z:.2f}'
                            )

                # =========================
                # 显示标签
                # =========================
                cv2.putText(
                    frame,
                    label,
                    (x, max(y - 8, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    2
                )

        # =========================
        # 发布检测结果
        # =========================
        output_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        output_msg.header = msg.header

        self.image_pub.publish(
            output_msg
        )

    # =========================================================
    # 获取深度
    # =========================================================
    def get_depth(self, u, v):

        if self.latest_depth is None:
            return None

        height, width = self.latest_depth.shape[:2]

        if (
            u < 0 or
            u >= width or
            v < 0 or
            v >= height
        ):
            return None

        # 不只读取一个像素，
        # 而是读取中心附近 5x5 区域
        region = self.latest_depth[
            max(0, v - 2):min(height, v + 3),
            max(0, u - 2):min(width, u + 3)
        ]

        region = region.astype(np.float32)

        valid = region[
            np.isfinite(region) &
            (region > 0)
        ]

        if len(valid) == 0:
            return None

        depth = float(
            np.median(valid)
        )

        # Gazebo 深度通常以米为单位
        if depth > 20.0:
            depth = depth / 1000.0

        return depth

    # =========================================================
    # 像素 + 深度 → camera_optical_frame 三维坐标
    # =========================================================
    def pixel_to_3d(self, u, v, depth):

        if self.camera_info is None:
            return None

        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        if fx == 0 or fy == 0:
            return None

        x = (
            (u - cx) *
            depth /
            fx
        )

        y = (
            (v - cy) *
            depth /
            fy
        )

        z = depth

        point = PointStamped()

        point.header.frame_id = (
            'camera_optical_frame'
        )

        point.header.stamp = (
            self.get_clock().now().to_msg()
        )

        point.point.x = x
        point.point.y = y
        point.point.z = z

        return point

    # =========================================================
    # camera_optical_frame → map
    # =========================================================
    def camera_to_map(
        self,
        point_camera,
        image_header
    ):

        try:

        

            transform = self.tf_buffer.lookup_transform(
                'map',
                'camera_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=0.1
                )
            )

            point_map = tf2_geometry_msgs.do_transform_point(
                point_camera,
                transform
            )

            return point_map

        except Exception as e:

            self.get_logger().warn(
                f'TF camera -> map failed: {e}'
            )

            return None

    # =========================================================
    # 没有深度时的普通颜色检测
    # =========================================================
    def detect_colors(self, frame):

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        color_ranges = {

            'red': [
                (
                    (0, 100, 80),
                    (10, 255, 255)
                ),
                (
                    (170, 100, 80),
                    (179, 255, 255)
                )
            ],

            'green': [
                (
                    (35, 80, 50),
                    (85, 255, 255)
                )
            ],

            'blue': [
                (
                    (90, 80, 50),
                    (130, 255, 255)
                )
            ],

            'yellow': [
                (
                    (20, 80, 80),
                    (40, 255, 255)
                )
            ]
        }

        for color_name, ranges in color_ranges.items():

            mask = None

            for lower, upper in ranges:

                current_mask = cv2.inRange(
                    hsv,
                    lower,
                    upper
                )

                if mask is None:
                    mask = current_mask
                else:
                    mask = cv2.bitwise_or(
                        mask,
                        current_mask
                    )

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:

                area = cv2.contourArea(
                    contour
                )

                if area < 300:
                    continue

                x, y, w, h = cv2.boundingRect(
                    contour
                )

                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (255, 255, 255),
                    2
                )

                cv2.putText(
                    frame,
                    color_name,
                    (x, max(y - 8, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2
                )


def main(args=None):

    rclpy.init(args=args)

    node = ColorDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
