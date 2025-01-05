# rt1_assignment2_2

## Overview

This package contains the implementation of a node which control a mobile robot in ROS2 Foxy using Gazebo.

## Installation

First, make sure you have the robot_urdf package in your workspace.

You can clone it from github with this link :

https://github.com/CarmineD8/robot_urdf (branch ros2)

Then, clone the rt1_assignment2_2 repository into your ros2 workspace and compile with:

    colcon build

Then, source your workspace with:

    source install/setup.bash

## Usage

1. Launch the simulation:
    ```bash
    ros2 launch robot_urdf gazebo.launch.py
    ```
2. In another terminal, run the control node:
    ```bash
    ros2 run rt1_assignment2_2 control_node
    ```

Then, you will see the mobile robot moving like a snake in the square [0:10]X[0:10] in Gazebo.

## Nodes

### snake_mvt_node

This node is responsible for controlling the robot's movements based the odometry. The movement is inspired from the one made for the exercice 1 in ROS using turtlesim. 

#### Subscriber
- Topic : `/odom`  Receives the robot's odometry data.
- Type : `nav_msgs/Odometry`

#### Publisher
- Topic : `/cmd_vel`: Publishes velocity commands to the robot.
- Type : `geometry_msgs/twist`

## Author

- Paul Pham Dang - 7899827
- I used the docker image with ROS2 Foxy