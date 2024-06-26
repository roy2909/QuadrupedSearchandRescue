import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatusArray
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import random
from scipy.ndimage import binary_erosion
from enum import Enum

class State(Enum):
    IDLE=1
    EXPLORING=2
    MOVING=3

class Exploration(Node):
    def __init__(self):
        super().__init__('exploration')

        # Subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, 'map', self.map_update, 10)
        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray,
            '/follow_path/_action/status',
            self.goal_status,
            10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.position, 10)

        # Publishers
        self.goal_publisher = self.create_publisher(
            PoseStamped, 'goal_pose', 10)
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'frontier_markers', 10)

        # Variables
        self.map_data = None
        self.map_array = None
        self.goal_reached = True
        self.robot_x_pose = 0.0
        self.robot_y_pose = 0.0
        self.map_info = None
        self.visited_grid = None
        self.frontier_array = MarkerArray()
        self.previous_goal = None
        self.state = State.IDLE


        # Timer
        self.timer = self.create_timer(5, self.timer_callback)

    def map_update(self, msg):
        self.map_data = msg.data
        self.map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.frontier_array.markers = []

        # Downsample the map
        downsample_factor = 2  
        downsampled_height = msg.info.height // downsample_factor
        downsampled_width = msg.info.width // downsample_factor
        self.downsampled_map = np.zeros((downsampled_height, downsampled_width))

        for y in range(downsampled_height):
            for x in range(downsampled_width):
                start_y = y * downsample_factor
                end_y = (y + 1) * downsample_factor
                start_x = x * downsample_factor
                end_x = (x + 1) * downsample_factor
                # Take the average value in the downsampling region
                self.downsampled_map[y, x] = np.mean(self.map_array[start_y:end_y, start_x:end_x])

        # Perform erosion operation
        if self.downsampled_map is not None:
            struct_elem = np.ones((3, 3), dtype=int)  # Define a 3x3 structuring element 
            self.downsampled_map = binary_erosion(self.downsampled_map, structure=struct_elem).astype(int)

        if self.map_info is not None:
            self.visited_grid = np.zeros((downsampled_height, downsampled_width))



    def timer_callback(self):
        if self.state == State.IDLE:
            # Handle idle state
            self.state = State.EXPLORING # Transition to exploring state

        elif self.state == State.EXPLORING:
            # Handle exploring state
            if self.goal_reached:
                frontiers = self.detect_frontiers(map_info=self.map_info)
                goal = self.select_goal(frontiers, self.map_array)
                print("Selected Goal:", goal)
                self.publish_goal(goal)
                self.publish_frontier_markers(frontiers)
            else:
                print("Goal not reached, waiting.")
        elif self.state == State.MOVING:
            # Handle moving state
            if self.goal_reached:
                self.state = State.EXPLORING
        else:
            print(f"Unknown state: {self.state}")

    def goal_status(self, msg):
        if msg.status_list:
            current_status = msg.status_list[-1].status
            print("Current Status:", current_status)

            if current_status == 2:
                self.goal_reached = False
                print("Robot is still moving.")
            elif current_status == 4:
                self.goal_reached = True
                print("Goal reached.")
                self.state = State.EXPLORING # Transition to exploring state
            else:
                self.goal_reached = True

    def position(self, msg):
        self.robot_x_pose = msg.pose.pose.position.x
        self.robot_y_pose = msg.pose.pose.position.y

    def is_valid_cell(self, x, y, map_info):
        return 0 <= x < map_info.width and 0 <= y < map_info.height

    def near_obstacle(self, x, y, map_data, threshold, downsample_factor):
        height, width = map_data.shape

        # Adjust coordinates for downsampling
        downsampled_x = x // downsample_factor
        downsampled_y = y // downsample_factor

        # Create a bounding box around the current cell
        start_x = max(0, downsampled_x - threshold)
        end_x = min(width // downsample_factor, downsampled_x + threshold + 1)
        start_y = max(0, downsampled_y - threshold)
        end_y = min(height // downsample_factor, downsampled_y + threshold + 1)

        # Extract the submatrix corresponding to the bounding box
        neighborhood = map_data[start_y:end_y, start_x:end_x]

        # Check if any cell in the neighborhood has the value 100 (indicating a wall)
        return np.any(neighborhood == 100)


    def detect_frontiers(self, map_info):
        # self.frontier_array.markers = []
        frontiers = []

        if not self.goal_reached or self.map_array is None:
            return frontiers

        self.get_logger().info(
            f"Map info received: Resolution: {map_info.resolution}, Width: {map_info.width}, Height: {map_info.height}")

        for y in range(map_info.height):
            for x in range(map_info.width):
                if self.map_array[y, x] == -1:  # unknown cell
                    neighbors = self.map_array[y - 1:y + 2, x - 1:x + 2]

                    if 0 in neighbors:  # free cell
                        if self.is_valid_frontier(x, y, map_info):
                            frontiers.append((x, y))

        return frontiers




    def is_valid_frontier(self, x, y, map_info):
        if not self.is_valid_cell(x, y, map_info):
            return False

        if self.near_obstacle(x, y, self.downsampled_map, threshold=3, downsample_factor=2):
            return False

        if self.has_been_visited(x, y,downsample_factor=2):
            return False

        return True

    def publish_frontier_markers(self, frontiers):
        marker_id = 0

        for x, y in frontiers:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x * self.map_info.resolution + \
                self.map_info.origin.position.x
            marker.pose.position.y = y * self.map_info.resolution + \
                self.map_info.origin.position.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            self.frontier_array.markers.append(marker)
            marker_id += 1
            # Check if there are more than 100 markers
        while len(self.frontier_array.markers) > 100:
            # Delete the oldest markers until the list has exactly 100 markers
            self.frontier_array.markers.pop(0)


        # Publish the marker array
        self.marker_publisher.publish(self.frontier_array)

    def has_been_visited(self, x, y, downsample_factor):
        # Adjust coordinates for downsampling
        downsampled_x = x // downsample_factor
        downsampled_y = y // downsample_factor

        # Check if the coordinates are within the valid range
        if 0 <= downsampled_y < self.visited_grid.shape[0] and 0 <= downsampled_x < self.visited_grid.shape[1]:
            # Check if the cell has been visited before
            return self.visited_grid[downsampled_y, downsampled_x] > 0
        else:
            return False



    def update_visited_grid(self, x, y):
        # Convert robot's position to downsampled grid indices
        downsample_factor = 2
        grid_x = int((x - self.map_info.origin.position.x) / (self.map_info.resolution * downsample_factor))
        grid_y = int((y - self.map_info.origin.position.y) / (self.map_info.resolution * downsample_factor))

        # Update the visited grid when the robot visits a cell
        if 0 <= grid_x < self.visited_grid.shape[1] and 0 <= grid_y < self.visited_grid.shape[0]:
            self.visited_grid[grid_y, grid_x] = 1
        else:
            self.get_logger().warn("Robot position is outside the map boundary. Cannot update visited grid.")

    def select_goal(self, frontiers, map_data):
        valid_frontiers = []

        for x, y in frontiers:
            if not self.near_obstacle(x, y, map_data, threshold=3,downsample_factor=2):
                valid_frontiers.append((x, y))

        if valid_frontiers:
            goal_x, goal_y = random.choice(valid_frontiers)

            real_x = goal_x * self.map_info.resolution + self.map_info.origin.position.x
            real_y = goal_y * self.map_info.resolution + self.map_info.origin.position.y

            # Check if the new goal is the same as the previous one
            if (real_x, real_y) == self.previous_goal or (
                    self.robot_x_pose == real_x and self.robot_y_pose == real_y):
                print("Same goal as previous or same pose. Choosing another.")
                valid_frontiers.remove((goal_x, goal_y))
                if valid_frontiers:
                    mid_index = len(valid_frontiers) // 2
                    goal_x, goal_y = valid_frontiers[mid_index]
                    real_x = goal_x * self.map_info.resolution + self.map_info.origin.position.x
                    real_y = goal_y * self.map_info.resolution + self.map_info.origin.position.y
                else:
                    print("No valid alternative goals.")
                    return None, None

            self.previous_goal = (real_x, real_y)

            print("Valid Frontiers:", valid_frontiers)
            print("Selected Goal (Real Coordinates):", real_x, real_y)

            return real_x, real_y
        else:
            return None, None


    def publish_goal(self, goal):
        if self.map_info is not None and self.map_info.resolution is not None:
            # Publishing the goal
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position = Point(x=goal[0] * 10 * self.map_info.resolution,
                                        y=goal[1] * 10 * self.map_info.resolution,
                                        z=0.0)
            self.goal_publisher.publish(goal_msg)
            self.get_logger().info(
                f"Goal: {goal_msg.pose.position.x}, {goal_msg.pose.position.y}")

            self.state = State.MOVING  # Transition to moving to goal state
        else:
            self.get_logger().warn("Map information or resolution is not available. Cannot publish goal.")
            self.state = State.IDLE  # Transition to idle state



        # Publishing a marker for the goal
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = goal_msg.pose.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.4
        marker.scale.y = 0.2
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0


        # Publish the marker
        self.marker_publisher.publish(MarkerArray(markers=[marker]))



def main(args=None):
    rclpy.init(args=args)
    node = Exploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()