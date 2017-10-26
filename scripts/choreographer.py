#!/usr/bin/env python

import math
import rospy
import std_srvs.srv as SS
import turtlesim.srv as TS
from geometry_msgs.msg import Twist


def main():
        # Node initialization for ROS
        path_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.init_node('choreographer')

        # Service initialization
        clear_board = rospy.ServiceProxy('/clear', SS.Empty)
        reset = rospy.ServiceProxy('/turtle1/teleport_absolute', TS.TeleportAbsolute)

        # Parameter setting & server connection
        if not rospy.has_param('~Speed'):
            rospy.set_param('~Speed', 10)

        velocity = Twist()    # Set up velocity container for path calculations
        update_rate = rospy.Rate(300)    # Set publisher rate to 300 Hz

        # Wait for teleport_absolute service to come available (no timeout)
        rospy.loginfo("Waiting for turtlesim reset service...")
        rospy.wait_for_service('/turtle1/teleport_absolute')

        # Reset turtle1 position to the center and orientation to 0 degrees theta
        # Clear the board so you do not pile up pen lines
        try:
            reset(5.5444,5.5444,0.5)
            clear_board()
        except rospy.ServiceException as exc:
            print("Turtle reset request failed :" + str(exc))

        # After resetting position, initialize the zero time to system time at reset
        time_init = rospy.Time.now()

        # Private parameter T_freq is set to user input to specify traversal rate
        T_freq = rospy.get_param('~Speed')
        rospy.loginfo("Turtle traversal rate (T) is set to %s.", T_freq)

        # Main calculation / command submission loop
        while not rospy.is_shutdown():

            time_cur = rospy.Time.now()
            time_elps = (time_cur - time_init).to_sec()

            dx = 12.0*math.pi*math.cos(4*math.pi*time_elps/T_freq)/T_freq
            dy = 6.0*math.pi*math.cos(2*math.pi*time_elps/T_freq)/T_freq

            theta = math.atan2(dy, dx)

            ddx = -48.0*(math.pi**2)*math.sin(4*math.pi*time_elps/T_freq)/(T_freq**2)
            ddy = -12.0*(math.pi**2)*math.sin(2*math.pi*time_elps/T_freq)/(T_freq**2)

            # Set the components of the velocity data structure
            velocity.linear.x = dx/math.cos(theta)
            velocity.angular.z = ((dx*ddy)-(ddx*dy))/((dx**2)+(dy**2))

            # Publish updated linear and angular velocities, then sleep until necessary
            path_pub.publish(velocity)
            update_rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass