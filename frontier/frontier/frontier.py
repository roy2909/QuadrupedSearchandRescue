import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatusArray
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalInfo
import random


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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Variables
        self.map_data = None
        self.map_array = None
        self.goal_reached = False
        self.robot_x_pose = 0.0
        self.robot_y_pose = 0.0
        self.map_info = None
        self.visited_grid = None
        self.frontier_array = MarkerArray()
        self.previous_goal = None

        # Timer
        self.timer = self.create_timer(5, self.timer_callback)

    def map_update(self, msg):
        self.map_data = msg.data
        self.map_array = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width))
        self.map_info = msg.info
        self.frontier_array.markers = []
        if self.map_info is not None:
            self.visited_grid = np.zeros(
                (self.map_info.height, self.map_info.width))

    def timer_callback(self):
        if self.map_array.any():
            if self.goal_reached:
                # print("Goal reached, setting nearby frontier.")
                frontiers = self.detect_frontiers(map_info=self.map_info)
                print("Detected Frontiers:", frontiers)

                goal = self.select_goal(frontiers, self.map_array)
                print("Selected Goal:", goal)
                self.publish_goal(goal)
            else:
                print("Goal not reached, waiting.")
                pass

    def goal_status(self, msg):
        if msg.status_list:
            current_status = msg.status_list[-1].status

            if current_status == 3:
                self.goal_reached = True
                print("Goal reached.")
            elif current_status == 2:
                self.goal_reached = False
                print("Active.")
            elif current_status == 4:
                self.goal_reached = True
                print("Goal aborted. Stopping the robot.")
            else:
                self.goal_reached = True

    def position(self, msg):
        self.robot_x_pose = msg.pose.pose.position.x
        self.robot_y_pose = msg.pose.pose.position.y

    def is_valid_cell(self, x, y, map_info):
        return 0 <= x < map_info.width and 0 <= y < map_info.height

    def near_obstacle(self, x, y, map_data, threshold):
        height, width = map_data.shape

        # Create a bounding box around the current cell
        start_x = max(0, x - threshold)
        end_x = min(width, x + threshold + 1)
        start_y = max(0, y - threshold)
        end_y = min(height, y + threshold + 1)

        # Extract the submatrix corresponding to the bounding box
        neighborhood = map_data[start_y:end_y, start_x:end_x]

        # Check if any cell in the neighborhood has the value 100 (indicating a wall)
        return np.any(neighborhood == 100)

    def detect_frontiers(self, map_info):
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

        if self.near_obstacle(x, y, self.map_array, threshold=3):
            return False

        if self.has_been_visited(x, y):
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
            marker.color.r = random.uniform(0.0, 1.0)
            marker.color.g = random.uniform(0.0, 1.0)
            marker.color.b = random.uniform(0.0, 1.0)

            self.frontier_array.markers.append(marker)
            marker_id += 1

        # Publish the marker array
        self.marker_publisher.publish(self.frontier_array)

    def has_been_visited(self, x, y):
        # Check if the cell has been visited before
        return self.visited_grid[y, x] > 0

    def update_visited_grid(self, x, y):
        # Update the visited grid when the robot visits a cell
        self.visited_grid[y, x] = self.get_clock().now().nanoseconds

    def select_goal(self, frontiers, map_data):
        valid_frontiers = []

        for x, y in frontiers:
            if not self.near_obstacle(x, y, map_data, threshold=3):
                valid_frontiers.append((x, y))

        if valid_frontiers:
            mid_index = len(valid_frontiers) // 2
            goal_x, goal_y = valid_frontiers[mid_index]

            real_x = goal_x * self.map_info.resolution + self.map_info.origin.position.x
            real_y = goal_y * self.map_info.resolution + self.map_info.origin.position.y

            # Check if the new goal is the same as the previous one
            if (real_x, real_y) == self.previous_goal:
                print("Same goal as previous. Choosing another.")
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
        # Publishing the goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position = Point(x=goal[0] * 10 * self.map_info.resolution,
                                       y=goal[1] * 10 *
                                       self.map_info.resolution,
                                       z=0.0)
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(
            f"Goal: {goal_msg.pose.position.x}, {goal_msg.pose.position.y}")


def main(args=None):
    rclpy.init(args=args)
    node = Exploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
