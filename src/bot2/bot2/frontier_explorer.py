
#!/usr/bin/env python3

import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException


class FrontierExplorer(Node):

    def __init__(self):

        super().__init__('frontier_explorer')

        # ============================================================
        # Parameters
        # ============================================================

        # frontier 区域最少包含多少个 frontier cell
        self.min_frontier_size = 5

        # 距离机器人太近的 frontier 不选择
        self.min_frontier_distance = 0.5

        # frontier 附近寻找 FREE goal 的最大半径
        self.goal_search_radius = 1.0

        # goal 周围安全距离
        # 防止目标贴着墙或障碍物
        self.goal_clearance = 0.35

        # 防止连续发送几乎相同的目标
        self.min_goal_change_distance = 0.5

        # ============================================================
        # State
        # ============================================================

        self.map_msg = None

        self.goal_active = False

        self.last_goal = None

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # Map QoS
        # ============================================================

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ============================================================
        # /map subscriber
        # ============================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        # ============================================================
        # Nav2 action client
        # ============================================================

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # ============================================================
        # Frontier visualization
        # ============================================================

        self.frontier_pub = self.create_publisher(
            Marker,
            '/exploration_frontiers',
            10
        )

        # ============================================================
        # Exploration timer
        # ============================================================

        self.timer = self.create_timer(
            2.0,
            self.exploration_loop
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Frontier Explorer started'
        )

        self.get_logger().info(
            'Robot frame: base_footprint'
        )

        self.get_logger().info(
            'Map frame: map'
        )

        self.get_logger().info(
            'Navigation: /navigate_to_pose'
        )

        self.get_logger().info(
            '========================================'
        )

    # ================================================================
    # Map callback
    # ================================================================

    def map_callback(self, msg):

        self.map_msg = msg

    # ================================================================
    # Get robot position in map frame
    # ================================================================

    def get_robot_position(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            return x, y

        except TransformException as ex:

            self.get_logger().warn(
                f'Cannot get map -> base_footprint TF: {ex}'
            )

            return None

    # ================================================================
    # Convert map cell to world coordinate
    # ================================================================

    def cell_to_world(self, row, col):

        resolution = self.map_msg.info.resolution

        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y

        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution

        return x, y

    # ================================================================
    # Convert world coordinate to map cell
    # ================================================================

    def world_to_cell(self, x, y):

        resolution = self.map_msg.info.resolution

        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y

        col = int(
            math.floor(
                (x - origin_x) / resolution
            )
        )

        row = int(
            math.floor(
                (y - origin_y) / resolution
            )
        )

        return row, col

    # ================================================================
    # Check map cell
    # ================================================================

    def valid_cell(self, row, col):

        height = self.map_msg.info.height
        width = self.map_msg.info.width

        return (
            0 <= row < height and
            0 <= col < width
        )

    # ================================================================
    # Check whether a FREE cell has enough clearance
    # ================================================================

    def is_safe_free_cell(
        self,
        data,
        row,
        col
    ):

        if not self.valid_cell(row, col):
            return False

        # 必须是已知 FREE
        if data[row, col] != 0:
            return False

        resolution = self.map_msg.info.resolution

        clearance_cells = max(
            1,
            int(
                math.ceil(
                    self.goal_clearance / resolution
                )
            )
        )

        height, width = data.shape

        r_min = max(
            0,
            row - clearance_cells
        )

        r_max = min(
            height - 1,
            row + clearance_cells
        )

        c_min = max(
            0,
            col - clearance_cells
        )

        c_max = min(
            width - 1,
            col + clearance_cells
        )

        # 在安全半径内不能出现障碍物
        for r in range(r_min, r_max + 1):

            for c in range(c_min, c_max + 1):

                if data[r, c] >= 50:
                    return False

        return True

    # ================================================================
    # Frontier detection
    #
    # Frontier 定义：
    #
    # UNKNOWN cell (-1)
    #     +
    # 至少相邻一个 FREE cell (0)
    #
    # 与原来的 FREE->UNKNOWN 定义相比，
    # 这里明确把 frontier 定义在未知区域边界。
    # ================================================================

    def find_frontier_mask(self, data):

        unknown = data == -1
        free = data == 0

        frontier = np.zeros_like(
            unknown,
            dtype=bool
        )

        # ------------------------------------------------------------
        # 上
        # ------------------------------------------------------------

        frontier[1:, :] |= (
            unknown[1:, :] &
            free[:-1, :]
        )

        # ------------------------------------------------------------
        # 下
        # ------------------------------------------------------------

        frontier[:-1, :] |= (
            unknown[:-1, :] &
            free[1:, :]
        )

        # ------------------------------------------------------------
        # 左
        # ------------------------------------------------------------

        frontier[:, 1:] |= (
            unknown[:, 1:] &
            free[:, :-1]
        )

        # ------------------------------------------------------------
        # 右
        # ------------------------------------------------------------

        frontier[:, :-1] |= (
            unknown[:, :-1] &
            free[:, 1:]
        )

        # ------------------------------------------------------------
        # 左上
        # ------------------------------------------------------------

        frontier[1:, 1:] |= (
            unknown[1:, 1:] &
            free[:-1, :-1]
        )

        # ------------------------------------------------------------
        # 右上
        # ------------------------------------------------------------

        frontier[1:, :-1] |= (
            unknown[1:, :-1] &
            free[:-1, 1:]
        )

        # ------------------------------------------------------------
        # 左下
        # ------------------------------------------------------------

        frontier[:-1, 1:] |= (
            unknown[:-1, 1:] &
            free[1:, :-1]
        )

        # ------------------------------------------------------------
        # 右下
        # ------------------------------------------------------------

        frontier[:-1, :-1] |= (
            unknown[:-1, :-1] &
            free[1:, 1:]
        )

        return frontier

    # ================================================================
    # BFS frontier clustering
    # ================================================================

    def cluster_frontiers(self, frontier_mask):

        height, width = frontier_mask.shape

        visited = np.zeros_like(
            frontier_mask,
            dtype=bool
        )

        clusters = []

        # 8 邻域
        neighbors = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, -1),
            (0, 1),
            (1, -1),
            (1, 0),
            (1, 1),
        ]

        rows, cols = np.where(frontier_mask)

        for start_row, start_col in zip(rows, cols):

            start_row = int(start_row)
            start_col = int(start_col)

            if visited[start_row, start_col]:
                continue

            queue = deque()

            queue.append(
                (start_row, start_col)
            )

            visited[start_row, start_col] = True

            cluster = []

            while queue:

                row, col = queue.popleft()

                cluster.append(
                    (row, col)
                )

                for dr, dc in neighbors:

                    nr = row + dr
                    nc = col + dc

                    if nr < 0 or nr >= height:
                        continue

                    if nc < 0 or nc >= width:
                        continue

                    if visited[nr, nc]:
                        continue

                    if not frontier_mask[nr, nc]:
                        continue

                    visited[nr, nc] = True

                    queue.append(
                        (nr, nc)
                    )

            # 删除太小的 frontier region
            if len(cluster) >= self.min_frontier_size:

                clusters.append(cluster)

        return clusters

    # ================================================================
    # Find safe FREE navigation goal around frontier cluster
    # ================================================================

    def find_goal_for_cluster(
        self,
        cluster,
        robot_x,
        robot_y
    ):

        if not cluster:
            return None

        resolution = self.map_msg.info.resolution

        # ============================================================
        # Frontier cluster 中心
        # ============================================================

        center_row = sum(
            row for row, col in cluster
        ) / len(cluster)

        center_col = sum(
            col for row, col in cluster
        ) / len(cluster)

        center_x, center_y = self.cell_to_world(
            int(round(center_row)),
            int(round(center_col))
        )

        # ============================================================
        # 地图数据
        # ============================================================

        data = np.array(
            self.map_msg.data,
            dtype=np.int8
        ).reshape(
            (
                self.map_msg.info.height,
                self.map_msg.info.width
            )
        )

        # ============================================================
        # 搜索范围
        # ============================================================

        max_radius_cells = max(
            1,
            int(
                self.goal_search_radius /
                resolution
            )
        )

        center_r = int(
            round(center_row)
        )

        center_c = int(
            round(center_col)
        )

        candidates = []

        for radius in range(
            0,
            max_radius_cells + 1
        ):

            r_min = center_r - radius
            r_max = center_r + radius

            c_min = center_c - radius
            c_max = center_c + radius

            for row in range(
                r_min,
                r_max + 1
            ):

                for col in range(
                    c_min,
                    c_max + 1
                ):

                    if not self.valid_cell(
                        row,
                        col
                    ):
                        continue

                    # 必须是安全 FREE cell
                    if not self.is_safe_free_cell(
                        data,
                        row,
                        col
                    ):
                        continue

                    x, y = self.cell_to_world(
                        row,
                        col
                    )

                    # =================================================
                    # 到 frontier 中心距离
                    # =================================================

                    frontier_distance = math.sqrt(
                        (x - center_x) ** 2 +
                        (y - center_y) ** 2
                    )

                    # =================================================
                    # 到机器人距离
                    # =================================================

                    robot_distance = math.sqrt(
                        (x - robot_x) ** 2 +
                        (y - robot_y) ** 2
                    )

                    # =================================================
                    # 排除机器人附近
                    # =================================================

                    if robot_distance < self.min_frontier_distance:
                        continue

                    candidates.append(
                        (
                            frontier_distance,
                            robot_distance,
                            x,
                            y,
                            row,
                            col
                        )
                    )

        if not candidates:
            return None

        # ============================================================
        # 选择最靠近 frontier 的安全 FREE cell
        # ============================================================

        candidates.sort(
            key=lambda item:
            item[0] +
            0.10 * item[1]
        )

        return candidates[0]

    # ================================================================
    # Choose best frontier region
    # ================================================================

    def choose_frontier_goal(
        self,
        clusters
    ):

        robot = self.get_robot_position()

        if robot is None:
            return None

        robot_x, robot_y = robot

        candidates = []

        for cluster in clusters:

            # ========================================================
            # frontier 中心
            # ========================================================

            center_row = sum(
                row for row, col in cluster
            ) / len(cluster)

            center_col = sum(
                col for row, col in cluster
            ) / len(cluster)

            center_x, center_y = self.cell_to_world(
                int(round(center_row)),
                int(round(center_col))
            )

            distance = math.sqrt(
                (center_x - robot_x) ** 2 +
                (center_y - robot_y) ** 2
            )

            if distance < self.min_frontier_distance:
                continue

            # ========================================================
            # 找安全 FREE navigation goal
            # ========================================================

            goal = self.find_goal_for_cluster(
                cluster,
                robot_x,
                robot_y
            )

            if goal is None:
                continue

            (
                frontier_goal_distance,
                robot_goal_distance,
                goal_x,
                goal_y,
                goal_row,
                goal_col
            ) = goal

            # ========================================================
            # 防止重复目标
            # ========================================================

            if self.last_goal is not None:

                last_x, last_y = self.last_goal

                goal_change = math.sqrt(
                    (goal_x - last_x) ** 2 +
                    (goal_y - last_y) ** 2
                )

                if goal_change < self.min_goal_change_distance:
                    continue

            # ========================================================
            # frontier size bonus
            #
            # 大 frontier 有更高探索价值
            # 但不让面积完全压过距离
            # ========================================================

            size_bonus = math.sqrt(
                float(len(cluster))
            )

            score = (
                robot_goal_distance
                + 0.5 * frontier_goal_distance
                - 0.20 * size_bonus
            )

            candidates.append(
                (
                    score,
                    distance,
                    len(cluster),
                    goal_x,
                    goal_y,
                    center_x,
                    center_y
                )
            )

        # ============================================================
        # 如果因为 last_goal 没有候选，
        # 第二次允许重新选择
        # ============================================================

        if not candidates:

            for cluster in clusters:

                center_row = sum(
                    row for row, col in cluster
                ) / len(cluster)

                center_col = sum(
                    col for row, col in cluster
                ) / len(cluster)

                center_x, center_y = self.cell_to_world(
                    int(round(center_row)),
                    int(round(center_col))
                )

                distance = math.sqrt(
                    (center_x - robot_x) ** 2 +
                    (center_y - robot_y) ** 2
                )

                if distance < self.min_frontier_distance:
                    continue

                goal = self.find_goal_for_cluster(
                    cluster,
                    robot_x,
                    robot_y
                )

                if goal is None:
                    continue

                (
                    frontier_goal_distance,
                    robot_goal_distance,
                    goal_x,
                    goal_y,
                    goal_row,
                    goal_col
                ) = goal

                size_bonus = math.sqrt(
                    float(len(cluster))
                )

                score = (
                    robot_goal_distance
                    + 0.5 * frontier_goal_distance
                    - 0.20 * size_bonus
                )

                candidates.append(
                    (
                        score,
                        distance,
                        len(cluster),
                        goal_x,
                        goal_y,
                        center_x,
                        center_y
                    )
                )

        if not candidates:
            return None

        # ============================================================
        # 选择最低 score
        # ============================================================

        candidates.sort(
            key=lambda item: item[0]
        )

        return candidates[0]

    # ================================================================
    # Publish frontier visualization
    # ================================================================

    def publish_frontiers(
        self,
        clusters
    ):

        if self.map_msg is None:
            return

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'frontier_explorer'

        marker.id = 0

        marker.type = Marker.CUBE_LIST

        marker.action = Marker.ADD

        resolution = self.map_msg.info.resolution

        marker.scale.x = resolution
        marker.scale.y = resolution
        marker.scale.z = 0.04

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 1.0

        for cluster in clusters:

            for row, col in cluster:

                x, y = self.cell_to_world(
                    row,
                    col
                )

                point = Point()

                point.x = x
                point.y = y
                point.z = 0.05

                marker.points.append(
                    point
                )

        self.frontier_pub.publish(
            marker
        )

    # ================================================================
    # Send Nav2 goal
    # ================================================================

    def send_goal(
        self,
        x,
        y
    ):

        if not self.nav_client.wait_for_server(
            timeout_sec=1.0
        ):

            self.get_logger().warn(
                'Nav2 /navigate_to_pose is not available.'
            )

            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # ============================================================
        # 默认朝向
        # ============================================================

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.goal_active = True

        self.last_goal = (
            x,
            y
        )

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            f'Navigation goal: '
            f'x={x:.2f}, y={y:.2f}'
        )

        self.get_logger().info(
            'Goal is a SAFE FREE map cell.'
        )

        self.get_logger().info(
            '----------------------------------------'
        )

        future = self.nav_client.send_goal_async(
            goal_msg
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # ================================================================
    # Goal response
    # ================================================================

    def goal_response_callback(
        self,
        future
    ):

        try:

            goal_handle = future.result()

        except Exception as ex:

            self.get_logger().error(
                f'Failed to send Nav2 goal: {ex}'
            )

            self.goal_active = False

            return

        if not goal_handle.accepted:

            self.get_logger().warn(
                'Nav2 rejected the goal.'
            )

            self.goal_active = False

            return

        self.get_logger().info(
            'Nav2 accepted the goal.'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.goal_result_callback
        )

    # ================================================================
    # Navigation result
    # ================================================================

    def goal_result_callback(
        self,
        future
    ):

        self.goal_active = False

        try:

            result = future.result()

            status = result.status

            if status == 4:

                self.get_logger().info(
                    'Goal reached successfully.'
                )

            elif status == 5:

                self.get_logger().warn(
                    'Goal was canceled.'
                )

            elif status == 6:

                self.get_logger().warn(
                    'Goal was aborted.'
                )

            else:

                self.get_logger().warn(
                    f'Navigation finished with status={status}'
                )

        except Exception as ex:

            self.get_logger().error(
                f'Failed to receive navigation result: {ex}'
            )

    # ================================================================
    # Main exploration loop
    # ================================================================

    def exploration_loop(self):

        # ============================================================
        # 等待地图
        # ============================================================

        if self.map_msg is None:
            return

        # ============================================================
        # 正在导航
        # ============================================================

        if self.goal_active:
            return

        # ============================================================
        # 获取地图
        # ============================================================

        height = self.map_msg.info.height
        width = self.map_msg.info.width

        data = np.array(
            self.map_msg.data,
            dtype=np.int8
        ).reshape(
            (
                height,
                width
            )
        )

        # ============================================================
        # 计算 frontier
        # ============================================================

        frontier_mask = self.find_frontier_mask(
            data
        )

        # ============================================================
        # frontier clustering
        # ============================================================

        clusters = self.cluster_frontiers(
            frontier_mask
        )

        # ============================================================
        # RViz visualization
        # ============================================================

        self.publish_frontiers(
            clusters
        )

        total_frontier_cells = int(
            frontier_mask.sum()
        )

        self.get_logger().info(
            f'Frontier cells: '
            f'{total_frontier_cells}, '
            f'frontier regions: '
            f'{len(clusters)}'
        )

        # ============================================================
        # 没有 frontier
        # ============================================================

        if not clusters:

            self.get_logger().info(
                'No frontier regions found.'
            )

            self.get_logger().info(
                'Exploration may be complete.'
            )

            return

        # ============================================================
        # 选择 frontier
        # ============================================================

        selected = self.choose_frontier_goal(
            clusters
        )

        if selected is None:

            self.get_logger().warn(
                'No valid navigation goal found.'
            )

            return

        (
            score,
            frontier_distance,
            frontier_size,
            goal_x,
            goal_y,
            frontier_x,
            frontier_y
        ) = selected

        # ============================================================
        # 输出 frontier 信息
        # ============================================================

        self.get_logger().info(
            'Selected frontier region:'
        )

        self.get_logger().info(
            f'  size     = {frontier_size}'
        )

        self.get_logger().info(
            f'  center   = '
            f'({frontier_x:.2f}, {frontier_y:.2f})'
        )

        self.get_logger().info(
            f'  distance = '
            f'{frontier_distance:.2f} m'
        )

        self.get_logger().info(
            f'  score    = '
            f'{score:.2f}'
        )

        self.get_logger().info(
            f'  goal     = '
            f'({goal_x:.2f}, {goal_y:.2f})'
        )

        # ============================================================
        # 发送真正的安全 FREE cell
        # ============================================================

        self.send_goal(
            goal_x,
            goal_y
        )


def main(args=None):

    rclpy.init(args=args)

    node = FrontierExplorer()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()


