## choreographer

This ROS package is designed to interface directly with a running `turtlesim` node. Upon runtime, this code will publish values for linear and angular velocity in order to influence an active turtle to follow a predetermined path. No input is necessary to utilize this package, however a Speed parameter can be passed to the node to override the default path traversal rate. 

The main script in this package is written primarily to broadcast velocity commands on a particular ROS topic. This script operates without any awareness of turtle state, and only considers system time for purposes of calculating solutions to kinematic equations. The script will run until terminated by an outside actor. 

A description of the interfaces in use, a quick breakdown of the code, and helpful tips for using this package immediately follow.


### Interfaces
##### Subscribers
- This node does not subscribe to any topics.

##### Publishers
- This node publishes linear and angular velocity values on the `/turtle1/cmd_vel` topic. The data format is of type **[geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)**.

##### Services
- This node leverages the `turtlesim/TeleportAbsolute` service to return the turtle to the center of the field.
- This node leverages the `std_srvs/Empty` service to clear the board of pen lines.

##### Parameters

To change the rate at which the turtle completes the loop, use the following command:

`rosrun choreographer choreographer.py _Speed:=xxx`

...where **xxx** is the value of the desired traversal rate. *NOTE:* Lower values will result in the turtle traversing the loop more quickly, whereas higher values will slow down the turtle. 


### Code Description

The [choreography.py](https://github.com/spieswl/choreographer/blob/master/scripts/choreographer.py) script first initializes all interfaces and establishes connections to the relevant service providers. The script waits until the `teleport_absolute` service becomes available and both resets and clears the simulator field.

After resetting the field, the code records the current system time and fetches a value from the parameter server for traversal rate (*default is 10*). If a new traversal rate is passed via `rosrun`, the code will print the registered value to the console before looping through the kinematic equation solutions.

The core functionality in the node is found on lines 45-65. For any time after `time_init`, the core loop will determine the time elapsed since initialization and perform the necessary operations to return floating point values for `linear.x` and `angular.z`. These elements correspond to the linear and angular velocities which the turtle will inherit after being published to the `cmd_vel` topic. The code is designed to publish new values at a rate of 300 Hz.

<img src="https://github.com/spieswl/choreographer/raw/master/images/turtle_figure8.png" width=360/>


### Helpful Inclusions
##### Launch File

`roslaunch choreographer chor_quickstart.launch`

##### Bag File

`bagfiles/trial_run.bag`

This bag file contains two complete circuits of the figure 8 trajectory that the turtle follows.


### References

\[1\] P. Morin and C. Samson, Springer Handbook of Robotics. Berlin: Springer, 2008, ch. Motion control of wheeled mobile robots, pp. 799 – 826
