import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from scipy.ndimage import binary_erosion
from enum import Enum
import numpy as np

class State(Enum):
    IDLE = 1
    EXPLORING = 2
    MOVING = 3

class Exploration(Node):
    """
    A ROS2 node for autonomous exploration using a frontier-based approach.
    """
    def __init__(self):
        super().__init__('exploration')

        # Subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, 'map', self.map_update, 10)
        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray, '/follow_path/_action/status', self.goal_status, 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.update_position, 10)

        # Publishers
        self.goal_publisher = self.create_publisher(
            PoseStamped, 'goal_pose', 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'frontier_markers', 10)

        # Variables
        self.map_data = None
        self.map_array = None
        self.map_info = None
        self.visited_grid = None
        self.downsampled_map = None
        self.state = State.IDLE
        self.goal_reached = True
        self.robot_x_pose = 0.0
        self.robot_y_pose = 0.0
        self.previous_goal = None
        self.frontier_array = MarkerArray()

        # Timer for periodic updates
        self.timer = self.create_timer(5.0, self.timer_callback)

    def map_update(self, msg):
        """Updates the map and processes it for exploration."""
        self.map_data = msg.data
        self.map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

        # Downsample the map
        downsample_factor = 2
        self.downsampled_map = self.downsample_map(self.map_array, downsample_factor)

        # Perform erosion on the downsampled map
        if self.downsampled_map is not None:
            struct_elem = np.ones((3, 3), dtype=int)
            self.downsampled_map = binary_erosion(self.downsampled_map, structure=struct_elem).astype(int)

        # Initialize the visited grid
        if self.map_info is not None:
            self.visited_grid = np.zeros_like(self.downsampled_map)

    def downsample_map(self, map_array, factor):
        """Downsamples the map by a given factor."""
        height, width = map_array.shape
        new_height, new_width = height // factor, width // factor
        downsampled = np.zeros((new_height, new_width))

        for y in range(new_height):
            for x in range(new_width):
                downsampled[y, x] = np.mean(map_array[y*factor:(y+1)*factor, x*factor:(x+1)*factor])

        return downsampled

    def timer_callback(self):
        """Handles periodic state updates."""
        if self.state == State.IDLE:
            self.state = State.EXPLORING

        elif self.state == State.EXPLORING:
            if self.goal_reached:
                frontiers = self.detect_frontiers()
                if frontiers:
                    goal = self.select_best_frontier(frontiers)
                    if goal:
                        self.publish_goal(goal)
                        self.publish_frontier_markers(frontiers)
                else:
                    self.get_logger().info("No frontiers detected.")
            else:
                self.get_logger().info("Waiting for goal to be reached.")

        elif self.state == State.MOVING:
            if self.goal_reached:
                self.state = State.EXPLORING

    def goal_status(self, msg):
        """Handles updates from the goal status topic."""
        if msg.status_list:
            current_status = msg.status_list[-1].status

            if current_status == 2:  # ACTIVE
                self.goal_reached = False
            elif current_status == 4:  # SUCCEEDED
                self.goal_reached = True
                self.state = State.EXPLORING

    def update_position(self, msg):
        """Updates the robot's current position."""
        self.robot_x_pose = msg.pose.pose.position.x
        self.robot_y_pose = msg.pose.pose.position.y
        self.update_visited_grid(self.robot_x_pose, self.robot_y_pose)

    def detect_frontiers(self):
        """Detects unexplored frontier cells in the map."""
        frontiers = []

        if self.map_array is None or self.map_info is None:
            return frontiers

        height, width = self.map_array.shape
        for y in range(height):
            for x in range(width):
                if self.map_array[y, x] == -1:  # Unknown cell
                    neighbors = self.map_array[max(0, y-1):y+2, max(0, x-1):x+2]
                    if 0 in neighbors:  # Adjacent to free space
                        if self.is_valid_frontier(x, y):
                            frontiers.append((x, y))

        return frontiers

    def is_valid_frontier(self, x, y):
        """Checks if a cell is a valid frontier."""
        if self.visited_grid is None or self.downsampled_map is None:
            return False

        if self.visited_grid[y//2, x//2] > 0:  # Adjust for downsampling
            return False

        return True

    def update_visited_grid(self, x, y):
        """Marks a cell as visited based on the robot's position."""
        if self.visited_grid is not None:
            grid_x = int((x - self.map_info.origin.position.x) / (self.map_info.resolution * 2))
            grid_y = int((y - self.map_info.origin.position.y) / (self.map_info.resolution * 2))

            if 0 <= grid_x < self.visited_grid.shape[1] and 0 <= grid_y < self.visited_grid.shape[0]:
                self.visited_grid[grid_y, grid_x] = 1

    def select_best_frontier(self, frontiers):
        """Selects the best frontier based on distance and exploration criteria."""
        if not frontiers:
            return None

        best_frontier = None
        min_cost = float('inf')

        for frontier in frontiers:
            fx, fy = frontier
            real_x = fx * self.map_info.resolution + self.map_info.origin.position.x
            real_y = fy * self.map_info.resolution + self.map_info.origin.position.y

            # Calculate distance cost
            distance_cost = np.hypot(real_x - self.robot_x_pose, real_y - self.robot_y_pose)

            # Add exploration weight to the cost
            exploration_cost = -distance_cost  

            total_cost = distance_cost + exploration_cost

            if total_cost < min_cost:
                min_cost = total_cost
                best_frontier = (real_x, real_y)

        return best_frontier

    def publish_goal(self, goal):
        """Publishes a new goal to the goal topic."""
        if goal:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position = Point(x=goal[0], y=goal[1], z=0.0)
            self.goal_publisher.publish(goal_msg)
            self.state = State.MOVING

    def publish_frontier_markers(self, frontiers):
        """Publishes markers for visualization of frontiers."""
        self.frontier_array.markers = []
        marker_id = 0

        for x, y in frontiers:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x * self.map_info.resolution + self.map_info.origin.position.x
            marker.pose.position.y = y * self.map_info.resolution + self.map_info.origin.position.y
            marker.pose.position.z = 0.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            self.frontier_array.markers.append(marker)
            marker_id += 1

        self.marker_publisher.publish(self.frontier_array)


def main(args=None):
    rclpy.init(args=args)
    exploration_node = Exploration()
    rclpy.spin(exploration_node)
    exploration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
