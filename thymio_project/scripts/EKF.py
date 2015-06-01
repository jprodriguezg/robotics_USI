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
from tf import TransformListener
from tf import Exception as TransformException
from robotics_lab_msgs.msg import EKF


tf_listener=None
time = None
#  Definition of some global variables!
CUBE_HALF_SIZE = 0.025
#  landmark detected info
nearest_landmark_id = -1.0
# Sensing output
zi = numpy.array ([[0.0],[0.0]])

# Callbacks

def ar_track_Callback(msg):

	#print("I'm in the callback")
	
	global nearest_landmark_id,zi

	flag = 0
	rho_ant = 1000
	for marker in msg.markers:
		flag = 1
		marker_id = marker.id
            	pose = marker.pose
            	pose.header.frame_id = marker.header.frame_id
		point=positionInRobotFrame(pose)
		if point:
			theta = math.atan2(point.y,point.x)
               		rho = math.sqrt(point.x*point.x +point.y*point.y)
			#print(marker_id)
			
			if rho < rho_ant:	# only takes the rho and theta of the nearest marker
				rho_ant = rho
				nearest_landmark_id=cube_id(marker_id)
				#print(rho)
				#print(theta)
				zi[0] = rho
				zi[1] = theta 	
				#print(zi)
	if flag == 0:
		nearest_landmark_id=-1

def OdomCallback(msg):

	global Ds, Dtheta, data_out, time, step

	dt = 1.0/10.0		# Frame rate of /odom topic
	if(time):
		dt = (msg.header.stamp-time).to_sec()
		step = msg.header.seq
		#print("real dt", dt)

	time = msg.header.stamp
	#dt = msg.header.stamp.nsecs-last_time
	Ds = msg.twist.twist.linear.x*dt
	Dtheta = msg.twist.twist.angular.z*dt	

	data_out.Odom = msg.pose.pose

# Helper Functions	
def normalizedAngle(da):

	da = math.fmod(da,2*math.pi)
	if da > math.pi :
		da=da-2*math.pi

	if da < -math.pi :
		da=da+2*math.pi

	return da

def positionInRobotFrame(face_pose_in_camera_frame):
        cube_pose_in_camera_frame=face_pose_in_camera_frame
        cube_pose_in_camera_frame.pose.position.z-=CUBE_HALF_SIZE
	global tf_listener 
	#listener = TransformListener(True)
	try:
		cube_pose_in_robot_frame = tf_listener.transformPose("base_link",cube_pose_in_camera_frame)	
            	return Point(cube_pose_in_robot_frame.pose.position.x,
                         cube_pose_in_robot_frame.pose.position.y,
                         cube_pose_in_robot_frame.pose.position.z)

	except TransformException as ex:
		rospy.loginfo("received an exception trying to transform from %s to base_link\n" \
                          % (face_pose_in_camera_frame.header.frame_id))
		print ex
		#return Point(0,0,0)
		return None
 
def cube_id(mark_id):

	i = 0
	real_id = -1
	while i <= 19:

		if mark_id>= i*6 and mark_id<i*6+5:
			real_id = i
		i = i+1
	return real_id 


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

def Fill_Publisher(out, xi, nearest_landmark_id, xlandmark, ylandmark, zi):

	out.header.stamp = rospy.Time.now()
	out.landmark_id = nearest_landmark_id
	out.rho = zi[0]
	out.theta = zi[1]
	out.landmark.x = xlandmark
	out.landmark.y = ylandmark
	out.EKF_prediction.position.x = xi[0]
	out.EKF_prediction.position.y = xi[1]
	out.EKF_prediction.orientation.z = xi[2]
	
	return out

# EKF functions
 
def prediction_update(xi,P,Vnoise,Ds,Dtheta):
	
	
	state_aux =numpy.array([Ds*numpy.cos(xi[2]+(Dtheta/2)), Ds*numpy.sin(xi[2]+(Dtheta/2)), Dtheta])

	
	xi = xi+state_aux				#-- state_aux is a 3X1 Matrix
	#print(xi[2])
	xi[2] = normalizedAngle(xi[2])

	x_comp = numpy.cos(xi[2]+Dtheta/2)
	y_comp = numpy.sin(xi[2]+Dtheta/2)
	
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
	r = numpy.sqrt(x_comp*x_comp+y_comp*y_comp)

	H_1 = numpy.array([[-x_comp/r,-y_comp/r,0],
		[y_comp/(r*r),-x_comp/(r*r),-1]])

	H_2 = numpy.array([[1,0],[0,1]])

	#S_1 = numpy.array([[1,0],[0,1]])
	S_1 = numpy.dot(H_1,P)
	S_1 = numpy.dot(S_1,H_1.transpose())

	#S_2 = numpy.array([[1,0],[0,1]])
	S_2 = numpy.dot(H_2,Wnoise)
	S_2 = numpy.dot(S_2,H_2.transpose())

	S = S_1+S_2

	G = numpy.dot(P,H_1.transpose()) 
	G = numpy.dot(G,inv(S))		#-- G is a 3X2 Matrix
	
	zi_in =numpy.array([[r], [math.atan2(y_comp,x_comp)-xi[2]]])
	inovation = zi-zi_in		#-- inovation is a 2X1 Matrix
	inovation[1] = normalizedAngle(inovation[1])
	#print(inovation)

	xi_aux = numpy.dot(G,inovation)
	xi_aux = xi_aux.transpose()
	
	# This avoids the transforamtion of xi (array)  to a matrix
	xi[0] = xi[0]+xi_aux[0][0]
	xi[1] = xi[1]+xi_aux[0][1]
	xi[2] = xi[2]+xi_aux[0][2]
	
	xi[2] = normalizedAngle(xi[2])

	aux_P = numpy.dot(G,H_1) 
	aux_P = numpy.dot(aux_P,P)	#-- aux_P is a 3X3 Matrix

	P = P-aux_P
	
	return xi, P
	

if __name__ == "__main__":

	rospy.init_node('EKF_node')

	#global tf_listener 
	tf_listener = TransformListener(True)

	rate = rospy.Rate(25)

	odom_subscriber=rospy.Subscriber('odom_input',Odometry,OdomCallback,queue_size=1)
	ar_track_subscriber=rospy.Subscriber('ar_track_input',AlvarMarkers,ar_track_Callback,queue_size=5)
	EKF_publisher = rospy.Publisher("EKF_output",EKF, queue_size=2)
	

	# Initialize odometry variables
	Ds = 0.0
	Dtheta = 0.0

	step = 0.0 
	last_step = 0.0

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
	data_out = EKF()

	while not rospy.is_shutdown():


		#print("I'm in the while")

		if step > last_step:
			xi, P = prediction_update(xi,P,Vnoise,Ds,Dtheta)
			last_step = step
			if nearest_landmark_id != -1:

				Wnoise[0][0] = 0.004*math.pow(zi[0][0],4)		# Computes the covariance matrix with rho of nearest marker
				#print(zi)
				(x_landmark,y_landmark) = cube_positions[nearest_landmark_id]
				xi, P = measurament_correction(xi,P,Wnoise,x_landmark,y_landmark,zi)
				data_out = Fill_Publisher(data_out,xi, nearest_landmark_id, x_landmark, y_landmark, zi)		
			else:
				data_out = Fill_Publisher(data_out,xi, -1, -1.0, -1.0, zi)
			# Publishing
			EKF_publisher.publish(data_out)

		# Control the rate of the node
		rate.sleep()
	rospy.signal_shutdown('Bye!')
	
