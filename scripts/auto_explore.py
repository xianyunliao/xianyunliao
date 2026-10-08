#!/usr/bin/env python3

import math
import heapq
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ============================================================
# 基本参数
# ============================================================

SDF_FILE = "/home/xianyun/turtlebot3_ws/install/bot2/share/bot2/worlds/bot2_room.sdf"

MIN_X = -7.8
MAX_X = 7.8
MIN_Y = -5.8
MAX_Y = 5.8

RESOLUTION = 0.20

# Burger 仿真机器人
ROBOT_RADIUS = 0.105

# 只留少量安全余量，避免把通道全部封死
SAFETY_MARGIN = 0.05

# 到达路径点的距离
WAYPOINT_TOLERANCE = 0.18

# 低速运行，优先保证 SLAM 稳定
MAX_LINEAR_SPEED = 0.10
MAX_ANGULAR_SPEED = 0.45

# 激光防撞
STOP_DISTANCE = 0.28
SLOW_DISTANCE = 0.60

CONTROL_PERIOD = 0.10


# ============================================================
# 工具
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def world_to_grid(x, y):
    gx = int(round((x - MIN_X) / RESOLUTION))
    gy = int(round((y - MIN_Y) / RESOLUTION))
    return gx, gy


def grid_to_world(gx, gy):
    x = MIN_X + gx * RESOLUTION
    y = MIN_Y + gy * RESOLUTION
    return x, y


# ============================================================
# 障碍物
# ============================================================

class BoxObstacle:

    def __init__(self, x, y, sx, sy, yaw):
        self.x = x
        self.y = y
        self.sx = sx
        self.sy = sy
        self.yaw = yaw


class CylinderObstacle:

    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


# ============================================================
# 读取 SDF
# ============================================================

def load_sdf_obstacles():

    tree = ET.parse(SDF_FILE)
    root = tree.getroot()

    boxes = []
    cylinders = []

    ignored_models = {
        "ground",
        "white_floor",
        "burger",
        "turtlebot3_burger",
        "robot",
    }

    for model in root.findall(".//model"):

        model_name = model.attrib.get("name", "")

        if model_name in ignored_models:
            continue

        pose = model.find("pose")

        if pose is None or pose.text is None:
            continue

        values = pose.text.strip().split()

        if len(values) < 6:
            continue

        try:
            x = float(values[0])
            y = float(values[1])
            yaw = float(values[5])
        except ValueError:
            continue

        collision = model.find(".//collision")

        if collision is None:
            continue

        geometry = collision.find("geometry")

        if geometry is None:
            continue

        box = geometry.find("box")

        if box is not None:

            size = box.find("size")

            if size is not None and size.text:

                values = size.text.strip().split()

                if len(values) >= 2:

                    boxes.append(
                        BoxObstacle(
                            x,
                            y,
                            float(values[0]),
                            float(values[1]),
                            yaw,
                        )
                    )

        cylinder = geometry.find("cylinder")

        if cylinder is not None:

            radius = cylinder.find("radius")

            if radius is not None and radius.text:

                cylinders.append(
                    CylinderObstacle(
                        x,
                        y,
                        float(radius.text),
                    )
                )

    return boxes, cylinders


# ============================================================
# 障碍物判断
# ============================================================

def inside_box(x, y, box):

    dx = x - box.x
    dy = y - box.y

    c = math.cos(-box.yaw)
    s = math.sin(-box.yaw)

    local_x = dx * c - dy * s
    local_y = dx * s + dy * c

    half_x = (
        box.sx / 2.0
        + ROBOT_RADIUS
        + SAFETY_MARGIN
    )

    half_y = (
        box.sy / 2.0
        + ROBOT_RADIUS
        + SAFETY_MARGIN
    )

    return (
        abs(local_x) <= half_x
        and
        abs(local_y) <= half_y
    )


def inside_cylinder(x, y, cylinder):

    distance = math.hypot(
        x - cylinder.x,
        y - cylinder.y,
    )

    return distance <= (
        cylinder.radius
        + ROBOT_RADIUS
        + SAFETY_MARGIN
    )


# ============================================================
# 建立地图
# ============================================================

def build_grid(boxes, cylinders):

    width = int(
        (MAX_X - MIN_X) / RESOLUTION
    ) + 1

    height = int(
        (MAX_Y - MIN_Y) / RESOLUTION
    ) + 1

    grid = [
        [False for _ in range(height)]
        for _ in range(width)
    ]

    for gx in range(width):

        for gy in range(height):

            x, y = grid_to_world(gx, gy)

            blocked = False

            for box in boxes:

                if inside_box(x, y, box):

                    blocked = True
                    break

            if blocked:

                grid[gx][gy] = True
                continue

            for cylinder in cylinders:

                if inside_cylinder(
                    x,
                    y,
                    cylinder,
                ):

                    blocked = True
                    break

            grid[gx][gy] = blocked

    return grid


# ============================================================
# A*
# ============================================================

def heuristic(a, b):

    return math.hypot(
        a[0] - b[0],
        a[1] - b[1],
    )


def astar(grid, start, goal):

    width = len(grid)
    height = len(grid[0])

    sx, sy = start
    gx, gy = goal

    if not (
        0 <= sx < width
        and
        0 <= sy < height
    ):
        return None

    if not (
        0 <= gx < width
        and
        0 <= gy < height
    ):
        return None

    if grid[sx][sy]:
        return None

    if grid[gx][gy]:
        return None

    open_set = []

    counter = 0

    heapq.heappush(
        open_set,
        (
            heuristic(start, goal),
            counter,
            start,
        ),
    )

    came_from = {}

    cost = {
        start: 0.0
    }

    directions = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
    ]

    while open_set:

        _, _, current = heapq.heappop(
            open_set
        )

        if current == goal:

            path = []

            while current in came_from:

                path.append(current)

                current = came_from[current]

            path.append(start)

            path.reverse()

            return path

        for dx, dy in directions:

            nx = current[0] + dx
            ny = current[1] + dy

            if not (
                0 <= nx < width
                and
                0 <= ny < height
            ):
                continue

            if grid[nx][ny]:
                continue

            # 防止斜向穿过墙角
            if dx != 0 and dy != 0:

                if grid[current[0] + dx][current[1]]:
                    continue

                if grid[current[0]][current[1] + dy]:
                    continue

            step = (
                math.sqrt(2.0)
                if dx != 0 and dy != 0
                else 1.0
            )

            neighbor = (nx, ny)

            new_cost = (
                cost[current]
                + step
            )

            if (
                neighbor not in cost
                or
                new_cost < cost[neighbor]
            ):

                cost[neighbor] = new_cost

                priority = (
                    new_cost
                    + heuristic(
                        neighbor,
                        goal,
                    )
                )

                came_from[neighbor] = current

                counter += 1

                heapq.heappush(
                    open_set,
                    (
                        priority,
                        counter,
                        neighbor,
                    ),
                )

    return None


# ============================================================
# 自动探索节点
# ============================================================

class AutoExplore(Node):

    def __init__(self):

        super().__init__("auto_explore")

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            20,
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            20,
        )

        self.x = None
        self.y = None
        self.yaw = None

        self.scan = None

        self.grid = None

        self.path = []
        self.path_index = 0

        self.goal = None
        self.goal_index = 0

        # 不预先假定每一个点都可达
        # A* 会自动验证
        self.targets = [
            (0.0, 2.0),
            (-4.5, 2.0),
            (-6.0, 1.0),
            (-6.0, -2.5),
            (-4.0, -2.8),
            (-1.0, -2.0),
            (2.0, -2.0),
            (4.5, -2.8),
            (6.0, -2.0),
            (6.0, 1.0),
            (4.5, 2.5),
            (1.5, 2.8),
        ]

        try:

            boxes, cylinders = (
                load_sdf_obstacles()
            )

            self.grid = build_grid(
                boxes,
                cylinders,
            )

            self.get_logger().info(
                "SDF obstacles loaded: "
                f"{len(boxes)} boxes, "
                f"{len(cylinders)} cylinders"
            )

        except Exception as error:

            self.get_logger().error(
                f"SDF loading failed: {error}"
            )

        self.timer = self.create_timer(
            CONTROL_PERIOD,
            self.control_loop,
        )

    # ========================================================
    # Odom
    # ========================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        sin_yaw = (
            2.0
            * (
                q.w * q.z
                + q.x * q.y
            )
        )

        cos_yaw = (
            1.0
            - 2.0
            * (
                q.y * q.y
                + q.z * q.z
            )
        )

        self.yaw = math.atan2(
            sin_yaw,
            cos_yaw,
        )

    # ========================================================
    # Laser
    # ========================================================

    def scan_callback(self, msg):

        self.scan = msg

    # ========================================================
    # 前方最小距离
    # ========================================================

    def front_distance(self):

        if self.scan is None:
            return float("inf")

        minimum = float("inf")

        for i, distance in enumerate(
            self.scan.ranges
        ):

            angle = (
                self.scan.angle_min
                +
                i * self.scan.angle_increment
            )

            if abs(angle) > math.radians(35):
                continue

            if not math.isfinite(distance):
                continue

            if distance < self.scan.range_min:
                continue

            if distance > self.scan.range_max:
                continue

            minimum = min(
                minimum,
                distance,
            )

        return minimum

    # ========================================================
    # 最近自由点
    # ========================================================

    def nearest_free(self, cell):

        if self.grid is None:
            return None

        width = len(self.grid)
        height = len(self.grid[0])

        gx, gy = cell

        if (
            0 <= gx < width
            and
            0 <= gy < height
            and
            not self.grid[gx][gy]
        ):
            return cell

        # 当前栅格如果被障碍物膨胀覆盖，
        # 在附近寻找最近的自由栅格。
        for radius in range(1, 16):

            candidates = []

            for dx in range(
                -radius,
                radius + 1,
            ):

                for dy in range(
                    -radius,
                    radius + 1,
                ):

                    if (
                        abs(dx) != radius
                        and
                        abs(dy) != radius
                    ):
                        continue

                    nx = gx + dx
                    ny = gy + dy

                    if not (
                        0 <= nx < width
                        and
                        0 <= ny < height
                    ):
                        continue

                    if self.grid[nx][ny]:
                        continue

                    candidates.append(
                        (
                            dx * dx + dy * dy,
                            nx,
                            ny,
                        )
                    )

            if candidates:

                candidates.sort(
                    key=lambda item: item[0]
                )

                return (
                    candidates[0][1],
                    candidates[0][2],
                )

        return None

    # ========================================================
    # 路径规划
    # ========================================================

    def plan(self, goal_x, goal_y):

        if self.x is None or self.y is None:
            return False

        start = world_to_grid(
            self.x,
            self.y,
        )

        goal = world_to_grid(
            goal_x,
            goal_y,
        )

        start = self.nearest_free(start)
        goal = self.nearest_free(goal)

        if start is None:

            self.get_logger().warn(
                "Cannot find free cell near robot."
            )

            return False

        if goal is None:

            self.get_logger().warn(
                f"Goal ({goal_x:.2f}, "
                f"{goal_y:.2f}) is blocked."
            )

            return False

        path = astar(
            self.grid,
            start,
            goal,
        )

        if path is None:

            self.get_logger().warn(
                f"A* failed: "
                f"({self.x:.2f}, "
                f"{self.y:.2f}) -> "
                f"({goal_x:.2f}, "
                f"{goal_y:.2f})"
            )

            return False

        self.path = [
            grid_to_world(px, py)
            for px, py in path
        ]

        self.path_index = 0

        self.goal = (
            goal_x,
            goal_y,
        )

        self.get_logger().info(
            f"Path found: "
            f"{len(self.path)} points -> "
            f"({goal_x:.2f}, "
            f"{goal_y:.2f})"
        )

        return True

    # ========================================================
    # 找下一个可达目标
    # ========================================================

    def next_goal(self):

        while (
            self.goal_index
            < len(self.targets)
        ):

            goal = self.targets[
                self.goal_index
            ]

            self.goal_index += 1

            if self.plan(
                goal[0],
                goal[1],
            ):

                return True

            self.get_logger().warn(
                f"Skip unreachable target: "
                f"{goal}"
            )

        self.get_logger().info(
            "Exploration finished."
        )

        self.stop_robot()

        return False

    # ========================================================
    # 主控制
    # ========================================================

    def control_loop(self):

        if (
            self.x is None
            or
            self.y is None
            or
            self.yaw is None
        ):
            return

        if self.grid is None:
            self.stop_robot()
            return

        # 第一次运行
        if not self.path:

            if not self.next_goal():
                return

        if (
            self.path_index
            >= len(self.path)
        ):

            self.path = []

            if not self.next_goal():
                return

            return

        # ----------------------------------------------------
        # 激光防撞
        # ----------------------------------------------------

        distance = self.front_distance()

        if distance <= STOP_DISTANCE:

            self.stop_robot()

            return

        # ----------------------------------------------------
        # 当前路径点
        # ----------------------------------------------------

        target_x, target_y = self.path[
            self.path_index
        ]

        dx = target_x - self.x
        dy = target_y - self.y

        distance_to_point = math.hypot(
            dx,
            dy,
        )

        # 到达路径点
        if distance_to_point < WAYPOINT_TOLERANCE:

            self.path_index += 1

            return

        target_yaw = math.atan2(
            dy,
            dx,
        )

        yaw_error = normalize_angle(
            target_yaw - self.yaw
        )

        cmd = Twist()

        # ----------------------------------------------------
        # 角速度
        # ----------------------------------------------------

        cmd.angular.z = clamp(
            1.2 * yaw_error,
            -MAX_ANGULAR_SPEED,
            MAX_ANGULAR_SPEED,
        )

        # ----------------------------------------------------
        # 线速度
        # ----------------------------------------------------

        # 偏角较大：
        # 原地调整方向，不边转边高速前进
        if abs(yaw_error) > math.radians(30):

            cmd.linear.x = 0.0

        else:

            speed = MAX_LINEAR_SPEED

            # 转向越大，速度越低
            speed *= max(
                0.25,
                1.0
                -
                abs(yaw_error) / 1.2,
            )

            # 前方障碍物接近时进一步减速
            if distance < SLOW_DISTANCE:

                factor = (
                    distance - STOP_DISTANCE
                ) / (
                    SLOW_DISTANCE
                    - STOP_DISTANCE
                )

                factor = clamp(
                    factor,
                    0.15,
                    1.0,
                )

                speed *= factor

            cmd.linear.x = speed

        self.cmd_pub.publish(cmd)

    # ========================================================
    # 停止
    # ========================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = AutoExplore()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.stop_robot()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
