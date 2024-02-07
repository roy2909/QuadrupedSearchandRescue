import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.qos import QoSProfile
import numpy as np
import random
import sys
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.map = OccupancyGrid()
        self.frontier_marker_pub = self.create_publisher(Marker,  "/visualization_marker", 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'goal_pose', QoSProfile(depth=1))
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, QoSProfile(depth=1))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def map_callback(self, msg):
        self.map = msg
        self.explore()

    def explore(self):
        self.get_logger().info("Exploring...")

        frontier_markers = MarkerArray()
        frontier_cells = self.find_frontier_cells()
        self.get_logger().info(f"Number of frontiers before check: {len(frontier_cells)}")

        if not frontier_cells:
            self.rotate_in_place()
            self.get_logger().info("No more frontiers.")
            return

        self.get_logger().info(f"Number of frontiers after check: {len(frontier_cells)}")

        robot_pose = self.get_robot_pose()


        closest_frontier_cell = None
        min_distance = float('inf')

        for cell in frontier_cells:
            if self.is_cell_within_map(cell):
                cell_pose = self.map_cell_to_pose(cell)
                distance = self.calculate_distance(robot_pose, cell_pose)
                if distance < min_distance:
                    min_distance = distance
                    closest_frontier_cell = cell

        if closest_frontier_cell:
            goal_pose = self.map_cell_to_pose(closest_frontier_cell)
            self.pose_pub.publish(goal_pose)

            marker = self.create_marker(closest_frontier_cell)
            frontier_markers.markers.append(marker)
            self.frontier_marker_pub.publish(frontier_markers)

    def rotate_in_place(self):
        rotation_goal = PoseStamped()
        rotation_goal.header.frame_id = 'map'
        rotation_goal.pose.orientation.z = 0.707
        rotation_goal.pose.orientation.w = 0.707
        self.pose_pub.publish(rotation_goal)

    def is_cell_within_map(self, cell):
        i, j = cell
        return 0 <= i < self.map.info.width and 0 <= j < self.map.info.height

    def calculate_distance(self, pose1, pose2):
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return np.sqrt(dx**2 + dy**2)

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
    
    def get_robot_pose(self, max_retries=3, retry_delay=1.0):
        retries = 0
        while retries < max_retries:
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                robot_pose = PoseStamped()
                robot_pose.header.frame_id = 'map'
                robot_pose.pose.position.x = transform.transform.translation.x
                robot_pose.pose.position.y = transform.transform.translation.y
                robot_pose.pose.position.z = transform.transform.translation.z
                robot_pose.pose.orientation = transform.transform.rotation
                return robot_pose
            except Exception as e:
                self.get_logger().warn(f"Failed to get robot pose: {str(e)}")
                retries += 1
                rclpy.sleep(retry_delay)

        self.get_logger().error(f"Failed to get robot pose after {max_retries} retries. Giving up.")
        return PoseStamped()


    def is_frontier_cell(self, index):
        if not (0 <= index < len(self.map.data)):
            return False  # Index out of range

        if self.map.data[index] == -1:  # Unknown cell
            neighbors = self.get_neighbors(index)
            for neighbor in neighbors:
                if not (0 <= neighbor < len(self.map.data)):
                    continue  # Skip neighbors outside the array
                if self.map.data[neighbor] == 0:  # Free cell
                    return True
        return False


    def get_neighbors(self, index):
        i, j = divmod(index, self.map.info.width)
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                ni, nj = i + dx, j + dy
                if 0 <= ni < self.map.info.width and 0 <= nj < self.map.info.height:
                    neighbors.append(ni * self.map.info.width + nj)
        return neighbors

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
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker

    def map_cell_to_pose(self, cell):
        pose = PoseStamped()
        pose.header.frame_id = self.map.header.frame_id
        pose.pose.position.x = cell[0] * self.map.info.resolution + self.map.info.origin.position.x
        pose.pose.position.y = cell[1] * self.map.info.resolution + self.map.info.origin.position.y
        pose.pose.orientation.w = 1.0
        return pose


def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()
    rclpy.spin(explorer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

