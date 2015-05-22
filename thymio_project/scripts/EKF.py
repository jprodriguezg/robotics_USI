#!/usr/bin/env python

import rospy
import numpy
import math
from numpy.linalg import inv
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def joyCallback(msg):
	x=0

def OdomCallback():
	x=0
	


def normalizedAngle(da):

	da=math.fmod(da,2*math.pi)
	if da > math.pi :
		da=da-2*math.pi

	if da < -math.pi :
		da=da+2*math.pi

	return da


def prediction_update(xi,P,Vnoise,Dd,Dangle):
	
	state_aux =numpy.array([(Dd)*math.cos(xi[2]+(Dangle/2)), 
						(Dd)*math.sin(xi[2]+(Dangle/2)),
						Dangle])

	xi = xi+state_aux				#-- state_aux is a 3X1 Matrix
	xi[2] = normalizedAngle(xi[2])

	x_comp = math.cos(xi[2]+Dangle/2)
	y_comp = math.sin(xi[2]+Dangle/2)

	F_1 = numpy.array([[1, 0, -Dd*y_comp],
		[0, 1, Dd*x_comp],
		[0, 0, 1]])

	F_2 = numpy.array([[x_comp,  -Dd*y_comp],
		[y_comp, Dd*x_comp],
		[0, 1]])

	P = numpy.dot(F_1,P)
	P = numpy.dot(P,F_1.transpose())
	
	aux_P = numpy.dot(F_2,Vnoise)
	aux_P = numpy.dot(aux_P,F_2.transpose())	#-- aux_P is an 3X3 matrix

	P = P + aux_P

	return P

def measurament_correction(xi,P,Wnoise,landmark_detected,zi):

	x_comp = landmark_detected[0]-xi[0]
	y_comp = landmark_detected[1]-xi[1]
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
	

if __name__ == "__main__":

	rospy.init_node('EKF_node')

	rate = rospy.Rate(20)

	joy_subscriber=rospy.Subscriber('joy_input',Joy,joyCallback,queue_size=1)
	odom_subscriber=rospy.Subscriber('odom_input',Twist,OdomCallback,queue_size=1)


	# Initialize odometry variables
	global Dd
	Dd = 0.0
	global Dangle
	Dangle = 0.0

	# Landmarks variables
	global landmark_detected
	landmark_detected = numpy.array ([0.0, 0.0, 0.0])

	# Initialize EFK varialbes
	global xi
	xi = numpy.array ([0.2, 0.2, math.pi/2.0])
	global P
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

	# Sensing output
	zi = numpy.array ([[0],[0]])
	
	while not rospy.is_shutdown():

		P = prediction_update(xi,P,Vnoise,Dd,Dangle)

		if landmark_detected[2] != -1:
			measurament_correction(xi,P,Wnoise,landmark_detected,zi)
		
		#print (P)

		# Control the rate of the node
		rate.sleep()
	rospy.signal_shutdown('Bye!')
	

	



