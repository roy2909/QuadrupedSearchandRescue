import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.qos import QoSProfile
import numpy as np
import random
import sys
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros as tf2
import math
import time
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.map = OccupancyGrid()
        self.marker_array_publisher = self.create_publisher(MarkerArray, '/frontier_cells', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'goal_pose', QoSProfile(depth=1))
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, QoSProfile(depth=1))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_pose = PoseStamped()
        self.last_robot_pose = PoseStamped()
        self.last_robot_pose_time = self.get_clock().now()
        self.last_frontier_cell = None
        self.same_frontier_count = 0
        self.max_same_frontier_count = 3
        

    def map_callback(self, msg):
        self.map = msg
        self.explore()

    def explore(self):
        frontier_cells = self.find_frontier_cells()
        marker_array = self.create_marker_array(frontier_cells)
        self.marker_array_publisher.publish(marker_array)

        if not frontier_cells:
            self.rotate_in_place()
            self.get_logger().info("No more frontiers.")
            return

        self.robot_pose = self.get_robot_pose()

        grouped_frontiers = self.group_frontiers(frontier_cells)
        if not grouped_frontiers:
            self.get_logger().info("No grouped frontiers.")
            return

        middle_frontier = self.find_middle_frontier(grouped_frontiers)
        if middle_frontier is not None:
            self.move_to_next_frontier(frontier_cells, middle_frontier)
            self.get_logger().info("Moving to the middle frontier.")

    def group_frontiers(self, frontier_cells):
        grouped_frontiers = []
        visited_cells = set()

        for cell in frontier_cells:
            if cell not in visited_cells:
                grouped_frontier = self.breadth_first_search(cell, frontier_cells)
                grouped_frontiers.append(grouped_frontier)
                visited_cells.update(grouped_frontier)

        return grouped_frontiers

    def breadth_first_search(self, start_cell, frontier_cells):
        queue = [start_cell]
        visited = set([start_cell])

        while queue:
            current_cell = queue.pop(0)

            for neighbor in self.get_neighbors(current_cell):
                if neighbor in frontier_cells and neighbor not in visited:
                    queue.append(neighbor)
                    visited.add(neighbor)

        return visited

    def find_middle_frontier(self, grouped_frontiers):
        if not grouped_frontiers:
            return None

        middle_frontier_group = grouped_frontiers[len(grouped_frontiers) // 3]
        middle_frontier_list = list(middle_frontier_group)

        if not middle_frontier_list:
            return None

        middle_frontier_index = len(middle_frontier_list) // 2
        middle_frontier = middle_frontier_list[middle_frontier_index]

        return middle_frontier

    def move_to_next_frontier(self, frontier_cells, middle_frontier):
        self.last_robot_pose_time = self.get_clock().now()
        next_frontier_cell = self.find_next_frontier(frontier_cells)

        if self.last_frontier_cell is not None and self.has_reached_goal():
            self.get_logger().info("Initial frontier completed. Moving to the next frontier.")
            self.last_frontier_cell = None
            time.sleep(1.0)

        if next_frontier_cell:
            goal_pose = self.map_cell_to_pose(next_frontier_cell)

            # Check if the goal is behind the robot
            angle_to_goal = self.calculate_angle_to_goal(goal_pose)
            if abs(angle_to_goal) > 90.0:
                self.get_logger().info("Goal is behind. Cancelling current goal.")
            else:
                self.pose_pub.publish(goal_pose)
                self.last_frontier_cell = next_frontier_cell

    def calculate_angle_to_goal(self, goal_pose):
        if self.robot_pose is None or goal_pose is None:
            return 0.0

        robot_yaw = self.get_yaw_from_pose(self.robot_pose)
        goal_yaw = self.get_yaw_from_pose(goal_pose)

        angle_to_goal = goal_yaw - robot_yaw

        if angle_to_goal > 180.0:
            angle_to_goal -= 360.0
        elif angle_to_goal < -180.0:
            angle_to_goal += 360.0

        return angle_to_goal
    
    def calculate_distance(self, pose1, pose2):
        if pose1 is None or pose2 is None:
            return float('inf')  # or any other appropriate value

        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return np.sqrt(dx**2 + dy**2)

    def get_yaw_from_pose(self, pose):
        orientation = pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return math.degrees(yaw)

    def has_reached_goal(self):
        if self.robot_pose is None or self.last_robot_pose is None:
            return False

        distance_to_goal = self.calculate_distance(self.robot_pose, self.last_robot_pose)

        if distance_to_goal < 0.3:
            self.get_logger().info("Robot has reached the goal.")
            return True
        else:
            self.get_logger().info("Robot has not reached the goal.")
            return False

    def find_next_frontier(self, frontier_cells):
        if self.last_frontier_cell:
            frontier_cells = [cell for cell in frontier_cells if cell != self.last_frontier_cell]

        if not frontier_cells:
            return None

        distances = [self.calculate_distance(self.robot_pose, self.map_cell_to_pose(cell)) for cell in frontier_cells]
        closest_index = distances.index(min(distances))

        return frontier_cells[closest_index]

    def rotate_in_place(self):
        rotation_goal = PoseStamped()
        rotation_goal.header.frame_id = 'map'
        rotation_goal.pose.orientation.z = 0.707
        rotation_goal.pose.orientation.w = 0.707
        self.pose_pub.publish(rotation_goal)

    def find_frontier_cells(self):
        self.get_logger().info("Finding frontier cells...")

        frontier_cells = []

        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                index = j * self.map.info.width + i
                if self.is_frontier_cell(index):
                    frontier_cells.append((i, j))

        self.get_logger().info(f"Number of frontier cells found: {len(frontier_cells)}")
        return frontier_cells

    def get_robot_pose(self, max_retries=5, retry_delay=1.0):
        retries = 0

        timeout = Duration(seconds=10.0)
        if not self.tf_buffer.can_transform('base_link', 'map', Time(), timeout):
            self.get_logger().warn("Failed to get robot pose: transform from 'base_link' to 'map' not available after waiting")
            return

        while retries < max_retries:
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', tf2.Time())
                robot_pose = PoseStamped()
                robot_pose.header.frame_id = 'map'
                robot_pose.pose.position.x = transform.transform.translation.x
                robot_pose.pose.position.y = transform.transform.translation.y
                robot_pose.pose.position.z = transform.transform.translation.z
                robot_pose.pose.orientation = transform.transform.rotation
                self.last_robot_pose = self.robot_pose
                return robot_pose
            except Exception as e:
                self.get_logger().warn(f"Failed to get robot pose: {str(e)}")
                retries += 1
                time.sleep(retry_delay)

        self.get_logger().error(f"Failed to get robot pose after {max_retries} retries. Giving up.")
        return PoseStamped()
    
    def get_neighbors(self, input):
        if isinstance(input, int):
            i, j = divmod(input, self.map.info.width)
        elif isinstance(input, tuple) and len(input) == 2:
            i, j = input
        else:
            raise ValueError("Invalid input format for get_neighbors")

        neighbors = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                ni, nj = i + dx, j + dy
                if 0 <= ni < self.map.info.width and 0 <= nj < self.map.info.height:
                    neighbors.append((ni, nj))

        return neighbors
    
    def is_frontier_cell(self, cell):
        if isinstance(cell, int):
            index = cell
        elif isinstance(cell, tuple) and len(cell) == 2:
            i, j = cell
            index = i * self.map.info.width + j
        else:
            raise ValueError("Invalid input format for is_frontier_cell")

        if not (0 <= index < len(self.map.data)):
            return False

        if self.map.data[index] == -1:
            neighbors = self.get_neighbors(cell)
            for neighbor in neighbors:
                ni, nj = neighbor
                if not (0 <= ni < self.map.info.height) or not (0 <= nj < self.map.info.width):
                    continue
                if self.map.data[ni * self.map.info.width + nj] == 0:
                    return True
        return False


    def create_marker(self, cell):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = self.map_cell_to_pose(cell).pose.position
        marker.scale.x = self.map.info.resolution
        marker.scale.y = self.map.info.resolution
        marker.scale.z = self.map.info.resolution
        marker.color.a = 1.0
        marker.color.r = random.random()
        marker.color.g = random.random()
        marker.color.b = random.random()
        return marker

    def create_marker_array(self, cells):
        marker_array = MarkerArray()
        for id, cell in enumerate(cells):
            marker = self.create_marker(cell)
            marker.id = id
            marker_array.markers.append(marker)
        return marker_array

    def map_cell_to_pose(self, cell):
        occupancy_value = self.map.data[cell[1] * self.map.info.width + cell[0]]

        if occupancy_value > 0:
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.map.header.frame_id
        pose.pose.position.x = cell[0] * self.map.info.resolution + self.map.info.origin.position.x
        pose.pose.position.y = cell[1] * self.map.info.resolution + self.map.info.origin.position.y
        pose.pose.orientation.w = 1.0
        return pose



    def euler_from_quaternion(self, quaternion):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()
    rclpy.spin(explorer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

