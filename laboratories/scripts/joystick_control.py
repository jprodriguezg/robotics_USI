#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def publish_velocities(out_velocities):
    velocities_publisher.publish(out_velocities)

def joyCallback(msg):
    out_velocities = Twist()

    out_velocities.linear.x = msg.axes[1]
    out_velocities.angular.z = msg.axes[3]

    publish_velocities(out_velocities)
	
def main():

    rospy.init_node('joystick_control')
    subscriber=rospy.Subscriber('joy_input',Joy,joyCallback,queue_size=1)

    global velocities_publisher
    velocities_publisher = rospy.Publisher("output_velocities",Twist, queue_size=2)

    rospy.spin()
   

if __name__ == "__main__":
    main() 
