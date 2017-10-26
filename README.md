ME495 Homework 1 Submission
========================

## choreographer - v1.0.0 ##

This ROS package is designed to interface directly with a running `turtlesim` node. Upon runtime, this code will publish values for linear and angular velocity in order to influence an active turtle to follow a predetermined path. No input is necessary to utilize this package, however a Speed parameter can be passed to the node to override the default path traversal rate. 

The main script in this package is written primarily to broadcast velocity commands on a particular ROS topic. This script operates without any awareness of turtle state, and only considers system time for purposes of calculating solutions to kinematic equations. The script will run until terminated by an outside actor. 

A description of the interfaces in use, a quick breakdown of the code, and helpful tips for using this package immediately follow.


### Interfaces ###
##### Subscribers #####
- This node does not subscribe to any topics.

##### Publishers ######
- This node publishes linear and angular velocity values on the `/turtle1/cmd_vel` topic. The data format is of type **[geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)**.

##### Services #####
- This node leverages the `turtlesim/TeleportAbsolute` service to return the turtle to the center of the field.
- This node leverages the `std_srvs/Empty` service to clear the board of pen lines.

##### Parameters #####

To change the rate at which the turtle completes the loop, use the following command:

`rosrun choreographer choreographer.py _Speed:=xxx`

...where **xxx** is the value of the desired traversal rate. *NOTE:* Lower values will result in the turtle traversing the loop more quickly, whereas higher values will slow down the turtle. 


### Code Description ###

The [choreography.py](https://github.com/ME495-EmbeddedSystems/homework-1-f2017-spieswl/blob/master/scripts/choreographer.py) script first initializes all interfaces and establishes connections to the relevant service providers. The script waits until the `teleport_absolute` service becomes available and both resets and clears the simulator field.

After resetting the field, the code records the current system time and fetches a value from the parameter server for traversal rate (*default is 10*). If a new traversal rate is passed via `rosrun`, the code will print the registered value to the console before looping through the kinematic equation solutions.

The core functionality in the node is found on [lines 45-65](https://github.com/ME495-EmbeddedSystems/homework-1-f2017-spieswl/blob/d9176828fea3b0ccd2ae59139a41233a2f083859/scripts/choreographer.py#L45-L65). For any time after `time_init`, the core loop will determine the time elapsed since initialization and perform the necessary operations to return floating point values for `linear.x` and `angular.z`. These elements correspond to the linear and angular velocities which the turtle will inherit after being published to the `cmd_vel` topic. The code is designed to publish new values at a rate of 300 Hz.

<img src="https://github.com/ME495-EmbeddedSystems/homework-1-f2017-spieswl/raw/master/images/turtle_figure8.png" width=360/>


### Helpful Inclusions ###
##### Launch File #####

`roslaunch choreographer chor_quickstart.launch`

##### Bag File #####

`bagfiles/trial_run.bag`

This bag file contains two complete circuits of the figure 8 trajectory that the turtle follows.


### References ###

\[1\] P. Morin and C. Samson, Springer Handbook of Robotics. Berlin: Springer, 2008, ch. Motion control of wheeled mobile robots, pp. 799 – 826


## ##


## ROS CLI Tools / Package Analysis Reponses ##


#### Topics ####
**Topic Names**: `/demo_publish_topic` and `/demo_subscriber_topic`
- While `ros_cl_demo` was running, I checked the topic list by entering `rostopic` in another byobu tab.

**Message Descriptions**:
- `/demo_publisher_topic` is of type *me495_hw1/ME495Pub*, a custom message included in the `me495_hw1` package.
  - This was determined by entering `rostopic info` for the publisher topic and checking the "type" field.
- `/demo_subscriber_topic` is of type *std_msgs/String*, which is included in std_msgs as part of the base ROS messages.
  - This was determined by entering `rostopic info` for the subscriber topic and checking the "type" field.

**Publishing Rate**: Exactly 50 Hz
- This was determined by entering `rostopic hz /demo_publisher_topic` and checking the average rate over multiple time windows.

**RQT Plot Analysis**: The plotted data in `rqt_plot` looks exactly like a trigonometric function with an amplitude of 10, and a frequency of ~2.
- I found this by simply opening `rqt_plot` via command line, specifying the `/demo_publish_topic` topic, and removing the time plot (which was messing with the automatic plot scaling).

**Callback Evaluation**: For a given input string "Testing" published to the `/demo_subscriber_topic` topic, the node reverses the text string and places a small message in front. "Testing" elicits "Manipulated String: gnitseT" in the terminal tab running the node.
- I published the string to the subscriber topic by entering the command `rostopic pub /demo_subscriber_topic std_msgs/String "Testing" -1` in a free byobu tab.


#### Service Providers ####
**Service Name**: `/me495_math_server`
- This was discovered by using the `rosservice list` command while the demo node was running. The `/get_loggers` and `/set_loggers_level` are included by default, so while those were created when the node was run, the math service is what has actual functionality.

**Service Definition**: The `math_server` service has a request message of name `input` and type uint32, while the response message is of name `output` and type uint8.
- This was found by using the `rossrv show me495_hw1/ME495Srv` command, after finding the name of the service by using `rossrv package me495_hw1`.

**Service Behavior**: This one was tricky. I resisted cracking open the code until I had called the `math_server` with a number of inputs just to give it a fair shot. Through the command `rosservice call /me495_math_server #` where # is some integer value, I was able to get the service to output a result...but that only returned values of 1, 1, 2, 3, and 5 for inputs of 1, 5, 10, 100, and 92929, respectively. I thought it was returning number of characters, but that didn't seem right (it is a math server, isn't it?) so I opened the CPP code and saw its just taking the base-10 log of the input and adding 1, if the input integer is greater than 0. If the integer is less than zero, it simply returns 1. 


### SSH Authentication ###

All set! I have been using SSH to interact with GitHub since the Hackathon.
