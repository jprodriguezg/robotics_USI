#!/usr/bin/env python

import rospy
import numpy
import math
import os

from numpy.linalg import inv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from robotics_lab_msgs.msg import path_follow
from robotics_lab_msgs.msg import EKF


# Initialize some global variables
xi = numpy.array ([0.2, 0.2, math.pi/2.0])

# Callbacks

def EKFCallback(msg):

	global xi
	xi[0] = msg.EKF_prediction.position.x	
	xi[1] = msg.EKF_prediction.position.y
	xi[2] = msg.EKF_prediction.orientation.z
	

# Helper Functions	
def normalizedAngle(da):

	da=math.fmod(da,2*math.pi)
	if da > math.pi :
		da=da-2*math.pi

	if da < -math.pi :
		da=da+2*math.pi

	return da

def read_path_points(filename_1, filename_2 ,waypoints,pose_frame):
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

        f=open(filename_1)
	i = 0
        for line in f.readlines():
	    i = i+1
            s=line.split(',')
            x=float(s[0])-pose_frame
            y=float(s[1])-pose_frame
            waypoints[i] = (x,y)
        f.close()
	
	print("Number of waypoints in file 1")
	i_aux = i
	print(i)
	f=open(filename_2)
        for line in f.readlines():
	    i = i+1
            s=line.split(',')
            x=float(s[0])-pose_frame
            y=float(s[1])-pose_frame
            waypoints[i] = (x,y)
        f.close()
	
	print("Number of waypoints in file 2")
	print(i-i_aux)
	# Save the number of waypoints
	numberofwaypoints = i

	return waypoints, numberofwaypoints

# Publisher functions
def FillPathFollowPublisher(out, waypoints_count, x_waypoint, y_waypoint, linear_vel, angular_vel, rho, beta, alpha):

	global xi
	out.header.stamp = rospy.Time.now()
	out.waypoint_id = waypoints_count
	out.rho = rho
	out.beta = beta
	out.alpha = alpha
	out.waypoint_pose.x = x_waypoint
	out.waypoint_pose.y = y_waypoint
	out.robot_pose.position.x = xi[0]
	out.robot_pose.position.y = xi[1]
	out.robot_pose.orientation.z = xi[2]
	out.control_vel.linear.x = linear_vel
	out.control_vel.angular.z = angular_vel

	return out

def FillVelocityPublisher(out, linear_vel, angular_vel):
	out.linear.x = linear_vel
	out.angular.z = angular_vel

	return out

# control functions

def velocity_limit(rho,linear_vel,angular_vel,linear_limit,angular_limit):

	delta_K = 0.01
	if rho < 0.06 and rho >= 0.04:
		linear_vel  = linear_vel + delta_K*5
	elif  rho < 0.04 and rho > 0.02: 
		linear_vel  = linear_vel + delta_K*2
	
	if linear_vel > linear_limit :
		linear_vel = linear_limit
	if linear_vel < -linear_limit :
		linear_vel = -linear_limit

	if angular_vel > angular_limit :
		angular_vel = angular_limit
	if angular_vel < -angular_limit:
		angular_vel = -angular_limit

	return linear_vel,angular_vel

def robot_control(xi,x_waypoint,y_waypoint, waypoints_count, numberofwaypoints):

	MindistanceToWayPoint = 0.04
	linear_limit = 0.02

	dx = x_waypoint-xi[0]
	dy = y_waypoint-xi[1]
	theta = xi[2]

	rho_control = math.sqrt(dx*dx+dy*dy)
	betha_control = math.atan2(dy,dx)
	alpha_control = betha_control-theta
	alpha_control = normalizedAngle(alpha_control)

	if math.fabs(alpha_control) < 0.78:	# ~=pi/8
		linear_vel = K_gain[0]*rho_control
		angular_limit = 0.2
	else:
		linear_vel = 0.005
		angular_limit = 0.2
	 
	angular_vel = K_gain[1]*alpha_control+K_gain[2]*betha_control

	linear_vel, angular_vel = velocity_limit(rho_control,linear_vel,angular_vel,linear_limit,angular_limit)

	if rho_control < MindistanceToWayPoint and waypoints_count<=numberofwaypoints:
		waypoints_count = waypoints_count+1

	return linear_vel, angular_vel, waypoints_count, rho_control, betha_control, alpha_control


if __name__ == "__main__":

	rospy.init_node('path_follow_node')

	rate = rospy.Rate(20)

	odom_subscriber=rospy.Subscriber('pose_input',EKF,EKFCallback,queue_size=1)
	vel_publisher = rospy.Publisher("velocities_output",Twist, queue_size=2)
	out_publisher = rospy.Publisher("path_follow_output",path_follow, queue_size=2)

	K_gain = numpy.array([3.0/2.0,8.0/2.5,-1.5/2.5])
	linear_vel = 0.0
	angular_vel = 0.0
	waypoints_count = 1

	# Creates the array with the path planned waypoints
	pose_frame = -0.9
	waypoints = {}
	path_planned_file_1 = rospy.get_param("~path_planned_file_1",-1)
	path_planned_file_2 = rospy.get_param("~path_planned_file_2",-1)
	waypoints, numberofwaypoints = read_path_points(path_planned_file_1, path_planned_file_2, waypoints,pose_frame)

	# Publisher objects
	velocity_out = Twist()
	path_follow_out = path_follow()
	

	while not rospy.is_shutdown():
		
		if waypoints_count<=numberofwaypoints:
			#print(waypoints_count)
			(x_waypoint,y_waypoint) = waypoints[waypoints_count]
			linear_vel, angular_vel, waypoints_count, rho, beta, alpha = robot_control(xi,x_waypoint,y_waypoint, waypoints_count, numberofwaypoints)

		else:
			linear_vel = 0.0
			angular_vel = 0.0


		# Publishing
		velocity_out = FillVelocityPublisher(velocity_out, linear_vel, angular_vel)	
		vel_publisher.publish(velocity_out)	

		path_follow_out = FillPathFollowPublisher(path_follow_out, waypoints_count, x_waypoint, y_waypoint, linear_vel, angular_vel, rho, beta, alpha)
		out_publisher.publish(path_follow_out)	

		# Control the rate of the node
		rate.sleep()
	rospy.signal_shutdown('Bye!')
