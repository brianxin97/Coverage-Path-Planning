#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point, Quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import time

class GoalPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('goal_publisher', anonymous=True)

        # Get parameters from the parameter server
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.2)
        self.robot_size = rospy.get_param("~robot_size", 0.2)  # Robot size in meters
        self.obstacle_distance_threshold = rospy.get_param("~obstacle_distance_threshold", self.robot_size )  # Safe distance from obstacles
        self.max_goal_distance = rospy.get_param("~max_goal_distance", 6.0)  # Maximum possible distance between each goal
        self.current_goal = None  # The current goal pose
        self.path = []  # List to store the planned path
        self.index = 0  # Index of the current goal in the path
        self.current_pose = None  # The current pose of the robot
        self.last_pose = None  # The last recorded pose of the robot
        self.obstacle_detected = False  # Flag to indicate if an obstacle is detected
        self.obstacle_positions = []  # List to store positions of detected obstacles
        self.trajectory = []  # List to store the trajectory of the robot
        self.stopped_duration_threshold = 7.0  # Time threshold for considering the robot as stopped (seconds)
        self.stopped_distance_threshold = 0.01  # Distance threshold for considering the robot as stopped (meters)
        self.last_movement_time = time.time()  # Timestamp of the last movement
        self.first_goal_set = False  # Flag to indicate if the first goal has been set

        # Map information
        self.map_data = None
        self.map_resolution = None
        self.map_width = None
        self.map_height = None
        self.map_origin_x = None
        self.map_origin_y = None

        # Subscribers
        self.path_sub = rospy.Subscriber('/coverage_path', Path, self.path_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Publishers
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/goal_markers', Marker, queue_size=10)
        self.trajectory_pub = rospy.Publisher('/trajectory_markers', MarkerArray, queue_size=10)

        # Timer to check if the robot has stopped
        rospy.Timer(rospy.Duration(1), self.check_if_stopped)

    def map_callback(self, msg):
        """Callback function for the map topic"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

    def path_callback(self, msg):
        """Callback function for the path topic"""
        self.path = msg.poses
        self.index = 0

    def odom_callback(self, msg):
        """Callback function for the odometry topic"""
        self.current_pose = msg.pose.pose
        if self.first_goal_set:
            self.update_trajectory()
            self.check_goal_reached()

    def scan_callback(self, data):
        """Callback function for the laser scan topic"""
        try:
            self.obstacle_detected = False
            self.obstacle_positions = []
            ranges = np.array(data.ranges)
            angles = np.linspace(data.angle_min, data.angle_max, len(ranges))
            for distance, angle in zip(ranges, angles):
                if distance < self.obstacle_distance_threshold:
                    self.obstacle_detected = True
                    # Calculate the obstacle position
                    ox = self.current_pose.position.x + distance * np.cos(angle)
                    oy = self.current_pose.position.y + distance * np.sin(angle)
                    self.obstacle_positions.append((ox, oy))
            if self.obstacle_detected:
                rospy.loginfo("Obstacle detected")
        except Exception as e:
            rospy.logerr("Error in scan_callback: {}".format(e))

    def check_if_stopped(self, event):
        """Check if the robot has stopped moving for a specified duration"""
        if self.current_pose and self.last_pose:
            distance_moved = self.compute_distance_moved(self.current_pose.position, self.last_pose.position)
            current_time = time.time()
            if distance_moved < self.stopped_distance_threshold:
                if current_time - self.last_movement_time >= self.stopped_duration_threshold:
                    if not self.first_goal_set and self.path:
                        self.send_next_goal()
                        self.first_goal_set = True
            else:
                self.last_movement_time = current_time
        self.last_pose = self.current_pose

    def compute_distance_moved(self, current_position, last_position):
        """Compute the distance the robot has moved since the last check"""
        return np.linalg.norm(np.array([current_position.x, current_position.y]) - np.array([last_position.x, last_position.y]))

    def is_goal_feasible(self, goal_pose):
        """Check if the goal is at a safe distance from obstacles using the map information"""
        if self.map_data is None:
            return False

        goal_x, goal_y = goal_pose.position.x, goal_pose.position.y
        goal_grid_x = int((goal_x - self.map_origin_x) / self.map_resolution)
        goal_grid_y = int((goal_y - self.map_origin_y) / self.map_resolution)

        # Define the search area around the goal
        search_radius = int(self.obstacle_distance_threshold / self.map_resolution)
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                nx, ny = goal_grid_x + dx, goal_grid_y + dy
                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                    if self.map_data[ny, nx] > 0:  # Obstacle cell
                        distance = np.sqrt((dx * self.map_resolution) ** 2 + (dy * self.map_resolution) ** 2)
                        if distance < self.obstacle_distance_threshold:
                            return False
        return True

    def send_next_goal(self):
        """Send the next goal from the path to the move_base action server"""
        while self.index < len(self.path):
            goal_pose = self.path[self.index].pose
            if self.is_goal_feasible(goal_pose):
                if self.is_within_max_distance(goal_pose):
                    goal = MoveBaseActionGoal()
                    goal.header.frame_id = "map"
                    goal.header.stamp = rospy.Time.now()
                    goal.goal.target_pose.header.frame_id = "map"
                    goal.goal.target_pose.header.stamp = rospy.Time.now()
                    goal.goal.target_pose.pose = goal_pose

                    self.goal_pub.publish(goal)
                    rospy.loginfo(f"Published goal {self.index}: ({goal.goal.target_pose.pose.position.x}, {goal.goal.target_pose.pose.position.y})")
                    self.publish_marker(goal.goal.target_pose.pose, self.index, Marker.ADD)
                    self.current_goal = goal_pose
                    self.index += 1
                    return
                else:
                    # Create intermediate goals if the goal is too far
                    self.create_intermediate_goals(goal_pose)
            else:
                rospy.logwarn(f"Goal {self.index} is too close to an obstacle. Skipping...")
                self.index += 1

        rospy.loginfo("Coverage complete!")

    def is_within_max_distance(self, goal_pose):
        """Check if the goal is within the maximum distance from the robot"""
        dx = goal_pose.position.x - self.current_pose.position.x
        dy = goal_pose.position.y - self.current_pose.position.y
        distance = np.sqrt(dx**2 + dy**2)
        return distance <= self.max_goal_distance

    def create_intermediate_goals(self, goal_pose):
        """Create intermediate goals if the target goal is too far"""
        dx = goal_pose.position.x - self.current_pose.position.x
        dy = goal_pose.position.y - self.current_pose.position.y
        distance = np.sqrt(dx**2 + dy**2)
        steps = int(np.ceil(distance / self.max_goal_distance))
        for step in range(1, steps):
            intermediate_pose = Pose()
            intermediate_pose.position.x = self.current_pose.position.x + (dx / steps) * step
            intermediate_pose.position.y = self.current_pose.position.y + (dy / steps) * step
            intermediate_pose.orientation = goal_pose.orientation  # Maintain the orientation of the goal
            if self.is_goal_feasible(intermediate_pose):
                self.path.insert(self.index + step - 1, PoseStamped(pose=intermediate_pose))
                rospy.loginfo(f"Inserted intermediate goal at ({intermediate_pose.position.x}, {intermediate_pose.position.y})")

    def publish_marker(self, pose, marker_id, action):
        """Publish a visualization marker for the goal"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_markers"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = action
        marker.pose = pose
        marker.scale.x = 0.2
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        rospy.loginfo(f"Published marker {marker_id} with action {action}.")

    def delete_marker(self, marker_id):
        """Delete a visualization marker for the goal"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_markers"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)
        rospy.loginfo(f"Deleted marker {marker_id}.")

    def update_trajectory(self):
        """Update the trajectory visualization"""
        point = Point()
        point.x = self.current_pose.position.x
        point.y = self.current_pose.position.y
        point.z = self.current_pose.position.z
        self.trajectory.append(point)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation = Quaternion(0, 0, 0, 1)  # Identity quaternion
        marker.scale.x = self.robot_size
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.points = self.trajectory

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.trajectory_pub.publish(marker_array)

    def check_goal_reached(self):
        """Check if the current goal has been reached"""
        if self.current_goal and self.current_pose:
            dx = self.current_goal.position.x - self.current_pose.position.x
            dy = self.current_goal.position.y - self.current_pose.position.y
            distance = np.sqrt(dx**2 + dy**2)
            if distance < self.goal_tolerance:
                self.delete_marker(self.index - 1)  # Delete the marker for the reached goal
                self.send_next_goal()

if __name__ == '__main__':
    try:
        # Instantiate the GoalPublisher and start the ROS event loop
        gp = GoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Goal Publisher node terminated.")
