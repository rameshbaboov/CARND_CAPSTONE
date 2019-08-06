# Autonomous Car - Capstone Final project - System Integration Project

This is the final project in the Udacity Self-Driving Car Nanodegree. The project guides a real car to drive around the track, using Robot Operating System or ROS. The project implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint generation and following


## System Architecture Diagram
The following is a system architecture diagram showing the ROS nodes and topics used in the project.

![Screenshot](./images/final-project-ros-graph-v2.png)

## Code Structure

### tl_detector
This package contains the traffic light detection node: tl_detector.py. This node takes in data from the /image_color, /current_pose, and /base_waypoints topics and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic. The /current_pose topic provides the vehicle's current position, and /base_waypoints provides a complete list of waypoints the car will be following. The module implements both a traffic light detection node and a traffic light classification node. Traffic light detection is implemented in tl_detector.py, whereas traffic light classification is implemented in tl_classfier.py.

![Screenshot](./images/tl-detector-ros-graph.png)


### waypoint_updater
This package contains the waypoint updater node: waypoint_updater.py. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics, and publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.

![Screenshot](./images/waypoint-updater-ros-graph.png)

### twist_controller
Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node dbw_node.py and the file twist_controller.py, along with a pid and lowpass filter that you can use in your implementation. The dbw_node subscribes to the /current_velocity topic along with the /twist_cmd topic to receive target linear and angular velocities. Additionally, this node will subscribe to /vehicle/dbw_enabled, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the /vehicle/throttle_cmd, /vehicle/brake_cmd, and /vehicle/steering_cmd topics.

![Screenshot](./images/dbw-node-ros-graph.png)

