#!/usr/bin/env python

## Simple demo that checks the proximity sensors
## and uses leds to warn about the
## detection of an object

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range
from std_msgs.msg import ColorRGBA


class ProximityWarn():
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
        rospy.init_node("proximity_warn")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
        rospy.Subscriber("/proximity/center", Range, self.proximity_callback)        

        self.led_pub = rospy.Publisher('/led/body/top', ColorRGBA, queue_size=10)
        """ if we need to do things repeatedly, 
        set a reasonable rate
        """
        self.rate = rospy.get_param("~rate", 10)
        rospy.loginfo("using rate %d hz",self.rate)
        
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
            
    def proximity_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.range)
        if data.range < data.max_range:
            """ we detected something, set led red """
            self.led_pub.publish(ColorRGBA(1,0,0,0))
        else:
            self.led_pub.publish(ColorRGBA(0,0,1,0))


if __name__ == '__main__':
    try:
        ProximityWarn().spin()
    except rospy.ROSInterruptException:
        pass
