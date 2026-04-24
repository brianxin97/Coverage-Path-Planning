Topic: Trajectory Planning for Cleaning Robots

# Coverage Path Planning

The repository for Coverage Path Planning project. Use turtlebot3 burger robot to automatically build the map from 3D environment model and coverage it. Tested on ubuntu 20.04.

## install dependencies

```
sudo apt-get update
sudo apt-get install ros-<ros_distro>-nav-msgs
sudo apt-get install ros-<ros_distro>-geometry-msgs
sudo apt-get install ros-<ros_distro>-sensor-msgs
sudo apt-get install ros-<ros_distro>-tf
sudo apt-get install ros-<ros_distro>-move-base-msgs
sudo apt-get install ros-<ros_distro>-visualization-msgs
sudo apt-get install ros-<ros_distro>-explore-lite

sudo apt-get install ros-<ros_distro>-rviz

sudo apt-get install ros-<ros_distro>-gazebo-ros
sudo apt-get install ros-<ros_distro>-gazebo-ros-pkgs

pip3 install rospy numpy tf
```

## clone repository to your root directory

```
mkdir -p robot_ws/src 
cd ~/robot_ws/src
git clone ...
cd ..
catkin_make
```

## download turtlebot3

```
cd ~/robot_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

## quick start

```
cd ~/robot_ws
catkin_make
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch coverage_robot coverage_planning.launch
```

This will launch the process until the robot finishes coveraging.

**Ajust parameter**. You can also ajust a few parameters in launch file to improve the coverage performance of robot.

robot_size: Smaller the robot size, denser the coverage path.

goal_tolerance: When the distance between robot and current goal is within this threshold, send the next goal. 

obstacle_distance_threshold: Mainly use to avoid crashing due to too small distance between goals and obstacles. Smaller the treshold, closer the goal to the obstacle.





