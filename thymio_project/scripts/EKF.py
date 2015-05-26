#!/usr/bin/env python

import rospy
import numpy
import math
import os

from numpy.linalg import inv
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
from nav_msgs.msg import Odometry



#  Definition of some global variables!
CUBE_HALF_SIZE = 0.025
#  landmark detected info
landmark_detected = numpy.array ([0.0, 0.0, -1.0])
# Sensing output
zi = numpy.array ([[0],[0]])


# Callbacks

def ar_track_Callback(msg):
	
	global landmark_detected,zi

	flag = 0
	rho_ant = 1000
	for i in msg.markers:
		flag = 1
		marker_id = msg.markers[i].id
            	pose = msg.markers[i].pose
            	pose.header.frame_id = msg.markers[i].header.frame_id
		point=positionInRobotFrame(pose)
		#print(i)
		if point:
			theta = math.atan2(point.y,point.x)
               		rho = math.sqrt(point.x*point.x +point.y*point.y)

			if rho < rho_ant:	# only takes the rho and theta of the nearest marker
				rho_ant = rho
				landmark_detected[2]=marker_id
				zi[0][0] = rho
				zi[1][0] = theta 
	if flag == 0:
		landmark_detected[2]=-1
	

def OdomCallback(msg):

	global Ds, Dtheta

	dt = 1.0/100		# Frame rate of /odom topic
	#dt = msg.header.stamp.nsecs-last_time
	Ds = msg.twist.twist.linear.x*dt
	Dtheta = msg.twist.twist.angular.z*dt	
	
	#last_time =  msg.header.stamp.secs+float(msg.header.stamp.nsecs/1000000000.0)
	#print(msg.header.seq)
	#print(msg.header.stamp.secs)
	#print(msg.header.stamp.nsecs)
	#print(msg.header.stamp)
	#print(float(msg.header.stamp.secs+float(msg.header.stamp.nsecs/1000000000.0)))
		

# Helper Functions	
def normalizedAngle(da):

	da=math.fmod(da,2*math.pi)
	if da > math.pi :
		da=da-2*math.pi

	if da < -math.pi :
		da=da+2*math.pi

	return da

def positionInRobotFrame(face_pose_in_camera_frame):
        cube_pose_in_camera_frame=face_pose_in_camera_frame;
        cube_pose_in_camera_frame.pose.position.z-=CUBE_HALF_SIZE;
 
	tf = TransformListener(True)	
	cube_pose_in_robot_frame = tf.transformPose("base_link",cube_pose_in_camera_frame)
            
	return Point(cube_pose_in_robot_frame.pose.position.x,
		cube_pose_in_robot_frame.pose.position.y,
		cube_pose_in_robot_frame.pose.position.z)


def read_cube_positions(filename,cube_positions):
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
        f.close()	
	return cube_positions

# Publisher function

def Fill_Publisher(out,xi):

	out.header.stamp = rospy.Time.now()
	out.pose.position.x = xi[0]
	out.pose.position.y = xi[1]
	out.pose.orientation.z = xi[2]
	
	return out

# EKF functions
 
def prediction_update(xi,P,Vnoise,Ds,Dtheta):
	
	state_aux =numpy.array([(Ds)*math.cos(xi[2]+(Dtheta/2)), 
						(Ds)*math.sin(xi[2]+(Dtheta/2)),
						Dtheta])

	xi = xi+state_aux				#-- state_aux is a 3X1 Matrix
	xi[2] = normalizedAngle(xi[2])

	x_comp = math.cos(xi[2]+Dtheta/2)
	y_comp = math.sin(xi[2]+Dtheta/2)

	F_1 = numpy.array([[1, 0, -Ds*y_comp],
		[0, 1, Ds*x_comp],
		[0, 0, 1]])

	F_2 = numpy.array([[x_comp,  -Ds*y_comp],
		[y_comp, Ds*x_comp],
		[0, 1]])

	P = numpy.dot(F_1,P)
	P = numpy.dot(P,F_1.transpose())
	
	aux_P = numpy.dot(F_2,Vnoise)
	aux_P = numpy.dot(aux_P,F_2.transpose())	#-- aux_P is an 3X3 matrix

	P = P + aux_P

	return xi, P

def measurament_correction(xi,P,Wnoise,x_landmark,y_landmark,zi):

	x_comp = x_landmark-xi[0]
	y_comp = y_landmark-xi[1]
	r = math.sqrt(x_comp*x_comp+y_comp*y_comp)

	H_1 = numpy.array([[-x_comp/r,-y_comp/r,0],
		[y_comp/(r*r),-x_comp/(r*r),-1]])

	H_2 = numpy.array([[1,0],[0,1]])

	S_1 = numpy.array([[1,0],[0,1]])
	S_1 = numpy.dot(H_1,P)
	S_1 = numpy.dot(S_1,H_1.transpose())

	S_2 = numpy.array([[1,0],[0,1]])
	S_2 = numpy.dot(H_2,Wnoise)
	S_2 = numpy.dot(S_2,H_2.transpose())

	S = S_1+S_2

	G = numpy.dot(P,H_1.transpose()) 
	G = numpy.dot(G,inv(S))		#-- G is a 2X3 Matrix
	
	zi_in =numpy.array([[r], [math.atan2(y_comp,x_comp)-xi[2]]])
	inovation = zi-zi_in		#-- inovation is a 2X1 Matrix
	inovation[1][0] = normalizedAngle(inovation[1][0])

	xi_aux = numpy.dot(G,inovation)
	xi = xi+xi_aux
	xi[0][2] = normalizedAngle(xi[0][2])

	aux_P = numpy.dot(G,H_1) 
	aux_P = numpy.dot(aux_P,P)	#-- aux_P is a 3X3 Matrix

	P = P-aux_P
	
	return xi, P
	

if __name__ == "__main__":

	rospy.init_node('EKF_node')

	rate = rospy.Rate(20)

	odom_subscriber=rospy.Subscriber('odom_input',Odometry,OdomCallback,queue_size=1)
	ar_track_subscriber=rospy.Subscriber('ar_track_input',AlvarMarkers,ar_track_Callback,queue_size=5)
	EKF_publisher = rospy.Publisher("EKF_output",PoseStamped, queue_size=2)
	

	# Initialize odometry variables
	Ds = 0.0
	Dtheta = 0.0

	# Landmarks variables
	
	cube_positions={}
	cube_pose_file = rospy.get_param("~cube_pose_file",-1)
	cube_positions = read_cube_positions(cube_pose_file,cube_positions)

	# Initialize EFK varialbes
	xi = numpy.array ([0.2, 0.2, math.pi/2.0])
	P = numpy.array ([[1, 0, 0],
		[0, 1, 0],
		[0, 0, 0.07]])

	nearest_rho = 0
	Wro = 0.004*math.pow(nearest_rho,4)
	Wbetha = 0.000006
	Wnoise = numpy.array ([[Wro, 0],[0, Wbetha]])

	Vd = 0.0000002
	Vangle = 0.01 
	Vnoise = numpy.array([[Vd, 0],[0,Vangle]])

	# EFK publish output
	data_out = PoseStamped()

	while not rospy.is_shutdown():

		xi, P = prediction_update(xi,P,Vnoise,Ds,Dtheta)

		if landmark_detected[2] != -1:
			(x_landmark,y_landmark) = cube_positions[landmark_detected[2]]
			xi, P = measurament_correction(xi,P,Wnoise,x_landmark,y_landmark,zi)

		# Publishing
		data_out = Fill_Publisher(data_out,xi)
		EKF_publisher.publish(data_out)

		# Control the rate of the node
		rate.sleep()
	rospy.signal_shutdown('Bye!')
	
