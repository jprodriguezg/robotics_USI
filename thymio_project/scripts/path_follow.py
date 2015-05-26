#!/usr/bin/env python

import rospy
import numpy
import math
import os

from numpy.linalg import inv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped


# Initialize some global variables
xi = numpy.array ([0.2, 0.2, math.pi/2.0])

# Callbacks

def EKFCallback(msg):

	global xi
	xi[0] = msg.pose.position.x	
	xi[1] = msg.pose.position.y
	xi[2] = msg.pose.orientation.z
	

# Helper Functions	
def normalizedAngle(da):

	da=math.fmod(da,2*math.pi)
	if da > math.pi :
		da=da-2*math.pi

	if da < -math.pi :
		da=da+2*math.pi

	return da

def read_path_points(filename,cube_positions):
        """ 
	format: one marker per line, marker_id x y
        marker_id: integer
        x,y: float (coordinate of marker in global frame in meters)
       
        if not os.path.isfile(filename, cube_positions):
            print "file ",filename,"does not exists"
            print "can not read markers positions"
            exit(1)
            return

	"""

        f=open(filename)
        for line in f.readlines():
            s=line.split()
            mid = int(s[0])
            x=float(s[1])
            y=float(s[2])
            cube_positions[mid] = (x,y)
	
	numberofwaypoints = f.readlines()
        f.close()
	
	return cube_positions, numberofwaypoints

# Publisher functions

def FillPathFollowPublisher(out):

	out.header.stamp = rospy.Time.now()
	out.pose.position.x = xi[0]
	out.pose.position.y = xi[1]
	out.pose.orientation.z = xi[2]
	
	return out

def FillVelocityPublisher(out, linear_vel, angular_vel):
	out.linear.x = linear_vel
	out.angular.z = angular_vel

# control functions

def velocity_limit(rho,linear_vel):

	delta_K = 0.01
	if rho < 0.06 and rho >= 0.04):
		linear_vel  = linear_vel + delta_K*5
	elif  rho < 0.04 and rho > 0.02: 
		linear_vel  = linear_vel + delta_K*2

	return linear_vel

def robot_control(xi,x_waypoint,y_waypoint, waypoints_count, numberofwaypoints):

	MindistanceToWayPoint = 0.02

	dx = x_waypoint-xi[0]
	dy = y_waypoint-xi[1]
	theta = xi[2]

	rho_control=math.sqrt(dx*dx+dy*dy)
	betha_control = math.atan2(dy,dx)
	alpha_control = betha_control-theta
	alpha_control = normalizedAngle(alpha_control)

	linear_vel = K_gain[0]*rho_control
	linear_vel = velocity_limit(rho_control,linear_vel)
	angular_vel = K_gain[1]*alpha_control+K_gain[2]*betha_control

	if rho_control < MindistanceToWayPoint and waypoints_count<=numberofwaypoints:
		waypoints_count = waypoints_count+1

	return linear_vel, angular_vel, waypoints_count


if __name__ == "__main__":

	rospy.init_node('path_follow_node')

	rate = rospy.Rate(20)

	odom_subscriber=rospy.Subscriber('pose_input',PoseStamped,EKFCallback,queue_size=1)
	vel_publisher = rospy.Publisher("velocities_output",Twist, queue_size=2)

	K_gain = numpy.array([3.0/2.0,8.0/2.5,-1.5/2.5])
	linear_vel = 0.0
	angular_vel = 0.0
	waypoints_count = 1

	# Creates the array with the path planned waypoints
	waypoints = {}
	path_planned_file = rospy.get_param("~path_planned_file",-1)
	waypoints, numberofwaypoints = read_path_points(path_planned_file,waypoints)

	# Publisher objects
	velocity_out = Twist()
	

	while not rospy.is_shutdown():
		
		if waypoints_count<=numberofwaypoints:
			(x_waypoint,y_waypoint) = waypoints[waypoints_count]
			linear_vel, angular_vel, waypoints_count = robot_control(xi,x_waypoint,y_waypoint, waypoints_count, numberofwaypoints)

		else:
			linear_vel = 0.0
			angular_vel = 0.0

		velocity_out = FillVelocityPublisher(velocity_out, linear_vel, angular_vel)	
		vel_publisher.publish(velocity_out)
			

		# Control the rate of the node
		rate.sleep()
	rospy.signal_shutdown('Bye!')
