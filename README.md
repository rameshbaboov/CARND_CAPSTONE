# Autonomous Car - Capstone Final project - System Integration Project

This is the final project in the Udacity Self-Driving Car Nanodegree. The project guides a real car to drive around the track, using Robot Operating System or ROS. The project implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint generation and following

## Project setup

The project was implemented using the below Architecture:

Docker on AWS:  A g3.4xlarge instance on AWS was used and Docker was built using Dockerfile. AMI ID - Deep Learning AMI (Amazon Linux) Version 22.0 (ami-01ac4e28da63bac3c)

## Thanks to OVERTAKERS for explaining how to run Docker on GPU. Without this, the project had multiple issues as the docker ran on CPU instead of GPU

## Docker Installation With Nvidia GPU

Build the docker image
```
docker build --rm . -f GPU.dockerfile -t capstone-gpu
```
Run the docker container
```
docker run --runtime=nvidia --rm -it -p 4567:4567  -v "/$(pwd)":/capstone -v /tmp/log:/root/.ros/ capstone-gpu
```
To log into same container using different session. This is required to check Topics and messages, play rosbag etc

List the container id
```
docker ps 
```
login to the container from other session
```
docker exec -it <container id> bash
```

Run in Simulator mode
```
source "/opt/ros/$ROS_DISTRO/setup.bash"
catkin_make
source devel/setup.bash
roslaunch launch/styx.launch 
```

Run in test mode

1. Download training bag that was recorded on the Udacity self-driving car.
2. Unzip the file - unzip traffic_light_bag_file.zip
3. Play the bag file - rosbag play -l traffic_light_bag_file/traffic_light_training.bag
4. Launch your project in site mode - cd programming_real_self_driving_car/ros
5. roslaunch launch/site.launch

Confirm that traffic light detection works on real life images

### Port forwarding
Since AWS AMI Ubuntu version does not have any GUI, the simulator was run on laptop and hence port forwarding had to be used. This was achieved by introducing option -L 4567:localhost:4567 in the ssh command given by AWS

ssh -i "xyz.pem" -L 4567:localhost:4567 ec2-user@ec2-18-234-233-113.compute-1.amazonaws.com

### ADDITIONAL INSTALLATIONS
Following additional installations were done:
1. Additional requirements were installed using requirements.txt - pip install -r requirements.txt
2. Pillow had to be updated as ROS launch was timing out - pip install Pillow
3. Rosdep update 


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

## Team

I could not join any team and hence did this project independently

## Rubric

1. The car should smoothly follows waypoints in the simulator - The waypoints are published and car follows smoothly
2. The Car should stop at traffic lights - The car stops at traffic lights
3. The DBW should publish commands for throttle, steering and brake at 50HZ
4. The car should work at different speeds. Tested with different values for kph velocity parameter in /ros/src/waypoint_loader/launch/waypoint_loader.launch. 
5. The vehicle should stop and restart PID controllers depending on the state of /vehicle/dbw_enabled.

## Project output



## Traffic light detection

The project uses the UNet Architecture and network has been trained on images created via simulator. Loss function is based on dice coefficient. The entire implementation can be found in tl_detector.py. The model folder has the retrained models. The Traffic_light_bag_files was used for training purpose

```
_______________________________________________________________________________________________
Layer (type)                     Output Shape          Param #     Connected to
====================================================================================================
input_1 (InputLayer)             (None, 96, 128, 1)    0
____________________________________________________________________________________________________
conv2d_1 (Conv2D)                (None, 96, 128, 32)   320         input_1[0][0]
____________________________________________________________________________________________________
conv_1_2 (Conv2D)                (None, 96, 128, 32)   9248        conv2d_1[0][0]
____________________________________________________________________________________________________
maxpool_1 (MaxPooling2D)         (None, 48, 64, 32)    0           conv_1_2[0][0]
____________________________________________________________________________________________________
conv_2_1 (Conv2D)                (None, 48, 64, 64)    18496       maxpool_1[0][0]
____________________________________________________________________________________________________
conv_2_2 (Conv2D)                (None, 48, 64, 64)    36928       conv_2_1[0][0]
____________________________________________________________________________________________________
maxpool_2 (MaxPooling2D)         (None, 24, 32, 64)    0           conv_2_2[0][0]
____________________________________________________________________________________________________
conv_3_1 (Conv2D)                (None, 24, 32, 128)   73856       maxpool_2[0][0]
____________________________________________________________________________________________________
conv_3_2 (Conv2D)                (None, 24, 32, 128)   147584      conv_3_1[0][0]
____________________________________________________________________________________________________
maxpool_3 (MaxPooling2D)         (None, 12, 16, 128)   0           conv_3_2[0][0]
____________________________________________________________________________________________________
conv_4_1 (Conv2D)                (None, 12, 16, 256)   295168      maxpool_3[0][0]
____________________________________________________________________________________________________
conv_4_2 (Conv2D)                (None, 12, 16, 256)   590080      conv_4_1[0][0]
____________________________________________________________________________________________________
maxpool_4 (MaxPooling2D)         (None, 6, 8, 256)     0           conv_4_2[0][0]
____________________________________________________________________________________________________
conv_5_1 (Conv2D)                (None, 6, 8, 512)     1180160     maxpool_4[0][0]
____________________________________________________________________________________________________
conv_5_2 (Conv2D)                (None, 6, 8, 512)     2359808     conv_5_1[0][0]
____________________________________________________________________________________________________
convtran_6 (Conv2DTranspose)     (None, 12, 16, 256)   524544      conv_5_2[0][0]
____________________________________________________________________________________________________
up_6 (Concatenate)               (None, 12, 16, 512)   0           convtran_6[0][0]
                                                                   conv_4_2[0][0]
____________________________________________________________________________________________________
conv_6_1 (Conv2D)                (None, 12, 16, 256)   1179904     up_6[0][0]
____________________________________________________________________________________________________
conv_6_2 (Conv2D)                (None, 12, 16, 256)   590080      conv_6_1[0][0]
____________________________________________________________________________________________________
convtran_7 (Conv2DTranspose)     (None, 24, 32, 128)   131200      conv_6_2[0][0]
____________________________________________________________________________________________________
up_7 (Concatenate)               (None, 24, 32, 256)   0           convtran_7[0][0]
                                                                   conv_3_2[0][0]
____________________________________________________________________________________________________
conv_7_1 (Conv2D)                (None, 24, 32, 128)   295040      up_7[0][0]
____________________________________________________________________________________________________
conv_7_2 (Conv2D)                (None, 24, 32, 128)   147584      conv_7_1[0][0]
____________________________________________________________________________________________________
convtran_8 (Conv2DTranspose)     (None, 48, 64, 64)    32832       conv_7_2[0][0]
____________________________________________________________________________________________________
up_8 (Concatenate)               (None, 48, 64, 128)   0           convtran_8[0][0]
                                                                   conv_2_2[0][0]
____________________________________________________________________________________________________
conv_8_1 (Conv2D)                (None, 48, 64, 64)    73792       up_8[0][0]
____________________________________________________________________________________________________
conv_8_2 (Conv2D)                (None, 48, 64, 64)    36928       conv_8_1[0][0]
____________________________________________________________________________________________________
convtran_9 (Conv2DTranspose)     (None, 96, 128, 32)   8224        conv_8_2[0][0]
____________________________________________________________________________________________________
up_9 (Concatenate)               (None, 96, 128, 64)   0           convtran_9[0][0]
                                                                   conv_1_2[0][0]
____________________________________________________________________________________________________
conv_9_1 (Conv2D)                (None, 96, 128, 32)   18464       up_9[0][0]
____________________________________________________________________________________________________
conv_9_2 (Conv2D)                (None, 96, 128, 32)   9248        conv_9_1[0][0]
____________________________________________________________________________________________________
conv2d_2 (Conv2D)                (None, 96, 128, 1)    33          conv_9_2[0][0]
====================================================================================================

```

### Traffic light classifier
The entire module is implemented in tl_classifier.py under light_classification subfolder. The images are classified into one of the predefined sets - Red, Yellow, Green and None. The accuracy on simulator images was 98% , but accuracy on real images were only 84%. Two different classifier models were used for this purpose and the models along with the weights can be found at the models folder under tl_detector

### How does the code work?

1. Waypoint Updater Node subscribes to /base_waypoints and /current_pose and publishes the final waypoints to /final_waypoints.
2. Once the /final_waypoints are published, the waypoint_follower node will start publishing messages to the/twist_cmd topic.
3. Traffic Light Detection is split into 2 parts:
   Detection: Detect the traffic light and its color from the /image_color. The topic /vehicle/traffic_lights contains the exact    location and status of all traffic lights in simulator
   Waypoint publishing: Once having identified the traffic light and determined its position, the code convert it to a waypoint    index and publish it.
 4. DBW_node.py (dbw_test in test mode) subscribes to /twist_cmd and use various controllers to provide appropriate throttle,       brake, and steering commands. These commands can then be published to the following topics:

      /vehicle/throttle_cmd
      /vehicle/brake_cmd
      /vehicle/steering_cmd

     Since a safety driver may take control of the car during testing, PID control is activated or deactivated by DBW status.        The DBW status is obtained by subscribing to /vehicle/dbw_enabled. When operating the simulator DBW can be toggled by             clicking "Manual" in the simulator GUI.

## Twist controller package files

### dbw_node.py
This file implements the dbw_node publishers and subscribers(current_velocity, /twist_cmd, and /vehicle/dbw_enabled topics). This file also imports the Controller class from twist_controller.py which is used for implementing the necessary controllers. The function used to publish throttle, brake, and steering is publish.

### twist_controller.py
This file contains the Controller class and implement vehicle control. Ths imports pid.py and lowpass.py to reduce acceleration noise and yaw_controller.py for steering. 

### yaw_controller.py
A controller  used to convert target linear and angular velocity to steering commands.

### pid.py
Implements PID controller used by twist_controller.py.

### lowpass.py
A generic low pass filter  used by twist_controller.py.

### dbw_test.py
Used to test DBW code against a bag recorded with a reference implementation. The bag can be found at https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/reference.bag.zip. The detailed use instructions was already given by Udacity in the same file.

### utilities given in the model code:

  1.get_waypoint_velocity(self, waypoint): gets the linear velocity (x-direction) for a single waypoint.

  2.set_waypoint_velocity(self, waypoints, waypoint, velocity): sets the linear velocity (x-direction) for a single waypoint in     a list of waypoints. Here, waypoints is a list of waypoints, waypoint is a waypoint index in the list, and velocity is the       desired velocity.

  3.distance(self, waypoints, wp1, wp2): Computes the distance between two waypoints in a list along the piecewise linear arc      connecting all waypoints between the two. Here, waypoints is a list of waypoints, and wp1 and wp2 are the indices of two waypoints in the list. This method may be helpful in determining the velocities for a sequence of waypoints leading up to a red light (the velocities should gradually decrease to zero starting some distance from the light)
