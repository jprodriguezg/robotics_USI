#!/usr/bin/env python

## Simple demo that checks the proximity sensors
## and uses leds to warn about the
## detection of an object

import rospy

from geometry_msgs.msg import Twist

class MoveIt():
    """ this class can also be used as a template for
    writing other nodes 
    """

    def __init__(self):
        """ here's a list of things we could do here:
        1- read parameters, e.g., 
        using rospy.get_param(), 
        2- call init_node
        3- subscribe to specific topics
        4 - create publishers
        ...
        """
        rospy.init_node("moving")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        """ if we need to do things repeatedly, 
        set a reasonable rate
        """
        self.rate = rospy.get_param("~rate", 10)
        rospy.loginfo("using rate %d hz",self.rate)
        self.move_until_ticks = 30
        self.x_speed = 0.1 # m/s
        
    def spin(self):
        """ here's the main (control) loop 
        """
        r = rospy.Rate(self.rate)

        
        while not rospy.is_shutdown():
            self.spin_once()
            r.sleep()

    def spin_once(self):
        """ this is executed at each contorl loop iteration
        """
        # create a twist message, fill in the details
        twist = Twist()
        twist.linear.x = self.x_speed;                   # our forward speed
        twist.linear.y = 0; twist.linear.z = 0;     # we can't use these!        
        twist.angular.x = 0; twist.angular.y = 0;   #          or these!
        twist.angular.z = 0;                        # no rotation

        # announce move, and publish the message
        rospy.loginfo("About to be moving forward!")
        self.vel_pub.publish(twist)
        self.move_until_ticks -= 1
        if self.move_until_ticks <= 0:
            twist = Twist()
            rospy.loginfo("About to stop")
            self.vel_pub.publish(twist)
    

if __name__ == '__main__':
    try:
        MoveIt().spin()
    except rospy.ROSInterruptException:
        pass
