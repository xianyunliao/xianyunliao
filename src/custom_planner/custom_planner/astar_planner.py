import heapq
import math

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # ==========================================================
        # /map QoS
        # 地图使用 Transient Local，确保能够收到已经发布的地图
        # ==========================================================
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ==========================================================
        # Subscribers
        # ==========================================================
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # ==========================================================
        # Publisher
        # ==========================================================
        self.path_pub = self.create_publisher(
            Path,
            '/plan',
            10
        )

        # ==========================================================
        # Map information
        # ==========================================================
        self.map_data = None

        self.map_width = 0
        self.map_height = 0

        self.map_resolution = 0.0

        self.map_origin_x = 0.0
        self.map_origin_y = 0.0

        self.map_received = False

        # ==========================================================
        # Robot odometry
        # ==========================================================
        self.robot_x = None
        self.robot_y = None

        self.robot_received = False

        # ==========================================================
        # Goal
        # ==========================================================
        self.goal_x = None
        self.goal_y = None

        self.goal_received = False

        # ==========================================================
        # Last generated path
        # ==========================================================
        self.last_path = None

        # ==========================================================
        # Timer
        #
        # 持续发布最后一次 A* 路径。
        # 这样 RViz 可以持续显示 /plan。
        # ==========================================================
        self.path_timer = self.create_timer(
            0.2,
            self.publish_last_path
        )

        # ==========================================================
        # Start information
        # ==========================================================
        self.get_logger().info(
            'A* Planner started.'
        )

        self.get_logger().info(
            'Waiting for /map, /odom and /goal_pose ...'
        )

    # ==============================================================
    # Map callback
    # ==============================================================

    def map_callback(self, msg):

        self.map_data = msg

        self.map_width = msg.info.width
        self.map_height = msg.info.height

        self.map_resolution = msg.info.resolution

        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        self.map_received = True

        self.get_logger().info(
            'Map received: '
            f'width={self.map_width}, '
            f'height={self.map_height}, '
            f'resolution={self.map_resolution:.3f} m'
        )

        self.get_logger().info(
            'Map origin: '
            f'x={self.map_origin_x:.3f}, '
            f'y={self.map_origin_y:.3f}'
        )

        # 地图只需要接收一次
        self.destroy_subscription(
            self.map_sub
        )

        self.get_logger().info(
            'Map loaded successfully.'
        )

    # ==============================================================
    # Odometry callback
    #
    # 注意：
    # 这里不再调用 try_plan()
    #
    # 防止 /odom 高频发布导致 A* 不断重复规划。
    # ==============================================================

    def odom_callback(self, msg):

        self.robot_x = (
            msg.pose.pose.position.x
        )

        self.robot_y = (
            msg.pose.pose.position.y
        )

        self.robot_received = True

    # ==============================================================
    # Goal callback
    #
    # 只有收到新的 /goal_pose 时才进行一次 A*
    # ==============================================================

    def goal_callback(self, msg):

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        self.goal_received = True

        self.get_logger().info(
            'New goal received: '
            f'x={self.goal_x:.3f}, '
            f'y={self.goal_y:.3f}'
        )

        self.try_plan()

    # ==============================================================
    # Try planning
    # ==============================================================

    def try_plan(self):

        if not self.map_received:
            self.get_logger().warn(
                'Cannot plan: map not received.'
            )
            return

        if not self.robot_received:
            self.get_logger().warn(
                'Cannot plan: odom not received.'
            )
            return

        if not self.goal_received:
            return

        # ----------------------------------------------------------
        # 当前机器人位置
        # ----------------------------------------------------------

        start = self.world_to_grid(
            self.robot_x,
            self.robot_y
        )

        # ----------------------------------------------------------
        # 目标位置
        # ----------------------------------------------------------

        goal = self.world_to_grid(
            self.goal_x,
            self.goal_y
        )

        self.get_logger().info(
            'Planning request: '
            f'start={start}, '
            f'goal={goal}'
        )

        # ----------------------------------------------------------
        # 检查起点
        # ----------------------------------------------------------

        if not self.is_free(
            start[0],
            start[1]
        ):
            self.get_logger().warn(
                f'Start cell {start} is not free.'
            )
            return

        # ----------------------------------------------------------
        # 检查目标
        # ----------------------------------------------------------

        if not self.is_free(
            goal[0],
            goal[1]
        ):
            self.get_logger().warn(
                f'Goal cell {goal} is not free.'
            )
            return

        # ----------------------------------------------------------
        # A* search
        # ----------------------------------------------------------

        self.get_logger().info(
            'Starting A* search...'
        )

        grid_path = self.astar(
            start,
            goal
        )

        if grid_path is None:

            self.get_logger().warn(
                'A* failed: no path found.'
            )

            return

        self.get_logger().info(
            'A* search successful!'
        )

        self.get_logger().info(
            f'Path length: '
            f'{len(grid_path)} cells'
        )

        # ----------------------------------------------------------
        # Publish path
        # ----------------------------------------------------------

        self.publish_path(
            grid_path
        )

    # ==============================================================
    # World coordinate -> Grid coordinate
    # ==============================================================

    def world_to_grid(
        self,
        x,
        y
    ):

        gx = int(
            (x - self.map_origin_x)
            / self.map_resolution
        )

        gy = int(
            (y - self.map_origin_y)
            / self.map_resolution
        )

        return gx, gy

    # ==============================================================
    # Grid coordinate -> World coordinate
    # ==============================================================

    def grid_to_world(
        self,
        gx,
        gy
    ):

        x = (
            self.map_origin_x
            + (gx + 0.5)
            * self.map_resolution
        )

        y = (
            self.map_origin_y
            + (gy + 0.5)
            * self.map_resolution
        )

        return x, y

    # ==============================================================
    # Check whether grid cell is free
    # ==============================================================

    def is_free(
        self,
        x,
        y
    ):

        # 越界
        if x < 0 or x >= self.map_width:
            return False

        if y < 0 or y >= self.map_height:
            return False

        index = (
            y * self.map_width
            + x
        )

        value = self.map_data.data[index]

        # 未知区域
        if value < 0:
            return False

        # 障碍物
        if value >= 50:
            return False

        # 空闲区域
        return True

    # ==============================================================
    # A* heuristic
    # ==============================================================

    def heuristic(
        self,
        a,
        b
    ):

        dx = (
            a[0] - b[0]
        )

        dy = (
            a[1] - b[1]
        )

        return math.sqrt(
            dx * dx
            + dy * dy
        )

    # ==============================================================
    # Get neighboring cells
    # ==============================================================

    def get_neighbors(
        self,
        current
    ):

        x, y = current

        neighbors = [
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),

            (x + 1, y + 1),
            (x + 1, y - 1),
            (x - 1, y + 1),
            (x - 1, y - 1)
        ]

        valid_neighbors = []

        for nx, ny in neighbors:

            if self.is_free(
                nx,
                ny
            ):

                valid_neighbors.append(
                    (nx, ny)
                )

        return valid_neighbors

    # ==============================================================
    # A* algorithm
    # ==============================================================

    def astar(
        self,
        start,
        goal
    ):

        open_set = []

        start_f = self.heuristic(
            start,
            goal
        )

        heapq.heappush(
            open_set,
            (
                start_f,
                start
            )
        )

        came_from = {}

        g_score = {
            start: 0.0
        }

        while open_set:

            _, current = (
                heapq.heappop(
                    open_set
                )
            )

            # ------------------------------------------------------
            # Goal reached
            # ------------------------------------------------------

            if current == goal:

                path = []

                while current in came_from:

                    path.append(
                        current
                    )

                    current = (
                        came_from[current]
                    )

                path.append(
                    start
                )

                path.reverse()

                return path

            # ------------------------------------------------------
            # Explore neighbors
            # ------------------------------------------------------

            for neighbor in self.get_neighbors(
                current
            ):

                dx = (
                    neighbor[0]
                    - current[0]
                )

                dy = (
                    neighbor[1]
                    - current[1]
                )

                # 对角线移动
                if (
                    dx != 0
                    and dy != 0
                ):

                    movement_cost = math.sqrt(
                        2.0
                    )

                else:

                    movement_cost = 1.0

                tentative_g = (
                    g_score[current]
                    + movement_cost
                )

                if (
                    neighbor not in g_score
                    or tentative_g
                    < g_score[neighbor]
                ):

                    came_from[
                        neighbor
                    ] = current

                    g_score[
                        neighbor
                    ] = tentative_g

                    f_score = (
                        tentative_g
                        + self.heuristic(
                            neighbor,
                            goal
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            f_score,
                            neighbor
                        )
                    )

        return None

    # ==============================================================
    # Convert A* grid path to ROS Path
    # ==============================================================

    def publish_path(
        self,
        grid_path
    ):

        path_msg = Path()

        # A* 路径使用 map 坐标系
        path_msg.header.frame_id = 'map'

        path_msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        total_distance = 0.0

        # ----------------------------------------------------------
        # Generate Path poses
        # ----------------------------------------------------------

        for i, (gx, gy) in enumerate(
            grid_path
        ):

            x, y = self.grid_to_world(
                gx,
                gy
            )

            pose = PoseStamped()

            pose.header.frame_id = 'map'

            pose.header.stamp = (
                path_msg.header.stamp
            )

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(
                pose
            )

            # ------------------------------------------------------
            # Calculate path distance
            # ------------------------------------------------------

            if i > 0:

                px, py = self.grid_to_world(
                    grid_path[i - 1][0],
                    grid_path[i - 1][1]
                )

                dx = x - px
                dy = y - py

                total_distance += math.sqrt(
                    dx * dx
                    + dy * dy
                )

        # ----------------------------------------------------------
        # Save the latest path
        # ----------------------------------------------------------

        self.last_path = path_msg

        # ----------------------------------------------------------
        # Publish immediately
        # ----------------------------------------------------------

        self.path_pub.publish(
            path_msg
        )

        self.get_logger().info(
            'A* path published to /plan.'
        )

        self.get_logger().info(
            f'Path distance: '
            f'{total_distance:.3f} m'
        )

    # ==============================================================
    # Continuously publish latest path
    # ==============================================================

    def publish_last_path(self):

        if self.last_path is None:
            return

        self.last_path.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        self.path_pub.publish(
            self.last_path
        )


# ==================================================================
# Main
# ==================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = AStarPlanner()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
