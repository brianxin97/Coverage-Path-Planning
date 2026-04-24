#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Twist
from sensor_msgs.msg import LaserScan
from collections import deque
from tf import TransformListener
import tf
import time
import heapq

class CoveragePathPlanner:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('coverage_path_planner', anonymous=True)

        # Parameters for robot and map
        self.resolution = None
        self.map_width = None
        self.map_height = None
        self.origin_x = None
        self.origin_y = None
        self.robot_size = rospy.get_param("~robot_size", 0.2)  # Robot size in meters
        self.map_data = None
        self.covered_cells = set()  # Set to store covered cells 
        self.path = []  # List to store the planned path
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.5)
        self.obstacle_distance_threshold = rospy.get_param("~obstacle_distance_threshold", 0.6)  # Obstacle distance threshold
        self.path_published = False 

        # ROS publishers and subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_pub = rospy.Publisher('/coverage_path', Path, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map" 

        # Transform listener for pose information
        self.tf_listener = tf.TransformListener()
        self.current_pose = None
        self.last_pose = None
        self.obstacle_detected = False

        # Variables to track if the robot has stopped
        self.last_movement_time = time.time()
        self.stopped_duration_threshold = 5.0  # seconds
        self.stopped_distance_threshold = 0.01  # Small movement threshold meters
        rospy.Timer(rospy.Duration(1), self.check_if_stopped)  # Timer to check if the robot has stopped
        rospy.loginfo("CoveragePathPlanner initialized.")
    
    def map_callback(self, data):
        """Callback function for the map topic"""
        try:
            # Extract map metadata
            self.resolution = data.info.resolution
            self.map_width = data.info.width
            self.map_height = data.info.height
            self.origin_x = data.info.origin.position.x
            self.origin_y = data.info.origin.position.y

            # Convert the map data to a NumPy array
            self.map_data = np.array(data.data).reshape((self.map_height, self.map_width))
            rospy.loginfo("Map received: resolution={}, width={}, height={}, origin=({}, {})".format(self.resolution, self.map_width, self.map_height, self.origin_x, self.origin_y))
            
            # Clear the set of covered cells
            self.covered_cells.clear()  # Clear the set of covered cells          
            
            # Convert map to a grid with reduced resolution
            self.grid_map = self.gridify_map(self.map_data, self.robot_size / self.resolution)        
            
            # Generate coverage path
            self.path = self.generate_coverage_path()
            
            # Publish path if not already published
            if not self.path_published:
                self.publish_path(self.path)
                rospy.loginfo("Map processed and path generated.")
            else:
                rospy.loginfo("Map processed but path not published due to robot being stationary.")
        except Exception as e:
            rospy.logerr("Error in map_callback: {}".format(e))
    
    def scan_callback(self, data):
        """Callback function for the laser scan topic"""
        try:
            self.obstacle_detected = False
            # Convert laser scan data to NumPy arrays for processing
            ranges = np.array(data.ranges)
            angles = np.linspace(data.angle_min, data.angle_max, len(ranges))
            # Check if any obstacle is within the threshold distance
            for distance, angle in zip(ranges, angles):
                if distance < self.obstacle_distance_threshold:
                    self.obstacle_detected = True
                    break
        except Exception as e:
            rospy.logerr("Error in scan_callback: {}".format(e))
    
    def odom_callback(self, data):
        """Callback function for the odometry topic"""
        self.current_pose = data.pose.pose

    def gridify_map(self, map_data, grid_size):
        """Convert the map to a coarser grid representation"""
        try:
            # Calculate the dimensions of the grid map
            grid_map = np.zeros((int(np.ceil(map_data.shape[0] / grid_size)), int(np.ceil(map_data.shape[1] / grid_size))), dtype=int)
            # Fill the grid map with the maximum value of each cell block
            for i in range(grid_map.shape[0]):
                for j in range(grid_map.shape[1]):
                    grid_map[i, j] = np.max(map_data[int(i*grid_size):int((i+1)*grid_size), int(j*grid_size):int((j+1)*grid_size)])
            rospy.loginfo("Grid map created with grid size: {}".format(grid_size))
            return grid_map
        except Exception as e:
            rospy.logerr("Error in gridify_map: {}".format(e))
            return None
        
    def generate_coverage_path(self):
        """Generate coverage path using a simple sweeping algorithm"""
        try:
            path = []
            # Traverse the grid map in a sweeping pattern
            for i in range(self.grid_map.shape[0] - 1, -1, -1):
                row_path = []
                for j in range(self.grid_map.shape[1] - 1, -1, -1):
                    if self.grid_map[i, j] == 0 and (i, j) not in self.covered_cells:
                        row_path.append((i, j))
                        self.covered_cells.add((i, j))
                # Reverse direction every other row 
                if (self.grid_map.shape[0] - 1 - i) % 2 == 1:
                    row_path.reverse()
                path.extend(row_path)
            rospy.loginfo("Coverage path generated with {} points.".format(len(path)))
            return self.plan_path_around_obstacles(path)
        except Exception as e:
            rospy.logerr("Error in generate_coverage_path: {}".format(e))
            return []

    def plan_path_around_obstacles(self, path):
        """Modify the path to avoid obstacles using D* Lite algorithm (basically A*)"""
        try:
            modified_path = []
            # Plan path between consecutive waypoints in the path
            for i in range(len(path) - 1):
                start = path[i]
                end = path[i + 1]
                modified_path.extend(self.d_star_lite(start, end))
            rospy.loginfo("Modified path to avoid obstacles with {} points.".format(len(modified_path)))
            return modified_path
        except Exception as e:
            rospy.logerr("Error in plan_path_around_obstacles: {}".format(e))
            return path

    def d_star_lite(self, start, goal):
        """D* Lite pathfinding algorithm"""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            _, current = heapq.heappop(open_set)          
            if current == goal:
                return self.reconstruct_path(came_from, current)           
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + self.distance(current, neighbor)              
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))      
        return []  # Return an empty path if no path is found

    def heuristic(self, a, b):
        """Heuristic function for D* Lite (Manhattan distance)"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def distance(self, a, b):
        """Distance function for D* Lite (Euclidean distance)"""
        return np.linalg.norm(np.array(a) - np.array(b))

    def get_neighbors(self, node):
        """Get neighbors for D* Lite"""
        neighbors = []
        # Check four possible neighbor positions (up, down, left, right)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (node[0] + dx, node[1] + dy)
            # Ensure the neighbor is within map bounds and not an obstacle
            if 0 <= neighbor[0] < self.grid_map.shape[0] and 0 <= neighbor[1] < self.grid_map.shape[1]:
                if self.grid_map[neighbor[0], neighbor[1]] == 0:  # Only consider non-obstacle cells
                    neighbors.append(neighbor)
        return neighbors

    def reconstruct_path(self, came_from, current):
        """Reconstruct path for D* Lite"""
        total_path = [current]
        # Backtrack from goal to start to reconstruct the path
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return total_path

    def publish_path(self, path):
        """Publish coverage path"""
        try:
            self.path_msg.header.stamp = rospy.Time.now()
            self.path_msg.poses = []
            for cell in path:
                x = self.origin_x + cell[1] * self.robot_size
                y = self.origin_y + cell[0] * self.robot_size
                # Boundary check to ensure the path point is within map bounds
                if x < self.origin_x or x > (self.origin_x + self.map_width * self.resolution) or y < self.origin_y or y > (self.origin_y + self.map_height * self.resolution):
                    rospy.logwarn("Generated path point ({}, {}) is out of map bounds.".format(x, y))
                    continue
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0  # Default orientation
                self.path_msg.poses.append(pose)
            self.path_pub.publish(self.path_msg)
            rospy.loginfo("Coverage path published with {} poses.".format(len(self.path_msg.poses)))
        except Exception as e:
            rospy.logerr("Error in publish_path: {}".format(e))

    def replan_path(self):
        """Replan path by updating the map and generating a new path without publishing it"""
        try:
            # Clear covered cells to ensure full coverage in new path
            self.covered_cells.clear()
            self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)
            self.publish_path(self.path)
            self.path_published = True
            rospy.loginfo("Replanned path but not published.")
        except Exception as e:
            rospy.logerr("Error in replan_path: {}".format(e))

    def check_if_stopped(self, event):
        """Check if the robot has stopped moving for a specified duration"""
        try:
            if self.current_pose and self.last_pose:
                distance_moved = self.compute_distance_moved(self.current_pose.position, self.last_pose.position)
                current_time = time.time()
                # Check if the robot has moved less than the threshold distance
                if distance_moved < self.stopped_distance_threshold:
                    if current_time - self.last_movement_time >= self.stopped_duration_threshold:
                        if not self.path_published:
                            rospy.loginfo("Robot has been stationary for 10 seconds. Disabling path publishing.")
                            #self.replan_path()
                            self.path_published = True
                else:
                    self.last_movement_time = current_time
                    #self.path_published = False  
            self.last_pose = self.current_pose
        except Exception as e:
            rospy.logerr("Error in check_if_stopped: {}".format(e))

    def compute_distance_moved(self, current_position, last_position):
        """Compute the distance the robot has moved since the last check"""
        return np.linalg.norm(np.array([current_position.x, current_position.y]) - np.array([last_position.x, last_position.y]))


if __name__ == '__main__':
    try:
        # Instantiate the planner and start the ROS event loop
        planner = CoveragePathPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Coverage Path Planner node terminated.")
