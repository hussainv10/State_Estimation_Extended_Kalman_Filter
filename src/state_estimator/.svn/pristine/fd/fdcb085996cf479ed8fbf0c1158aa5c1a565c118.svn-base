#!/usr/bin/env python

# Columbia Engineering
# MECS 4602 - Fall 2018

import math
import numpy
import time

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData

class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.01

        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
        
    # This function gets called every time the robot publishes its control 
    # input and sensory output. You must make use of what you know about 
    # extended Kalman filters to come up with an estimate of the current
    # state of the robot and covariance matrix.
    # The SensorData message contains fields 'vel_trans' and 'vel_ang' for
    # the commanded translational and rotational velocity respectively. 
    # Furthermore, it contains a list 'readings' of the landmarks the
    # robot can currently observe
    def estimate(self, sens):

        #### ----- YOUR CODE GOES HERE ----- ####

	#given formulae

	#range = math.sqrt( (xr-xl)*(xr-xl) + (yr-yl)*(yr-yl) )
	#bearing = math.atan2(yl-yr, xl-xr) - thetar

	#x(k+1) = x(k) + t * vel_trans * cos(theta)
	#y(k+1) = y(k) + t * vel_trans * sin(theta)
	#theta(k+1) = theta(k) + t * vel_ang


	
	x_hat = self.x[0] + self.step_size * sens.vel_trans * numpy.cos(self.x[2])
	y_hat = self.x[1] + self.step_size * sens.vel_trans * numpy.sin(self.x[2])
	theta_hat = self.x[2] + self.step_size * sens.vel_ang
	X_hat = numpy.array([x_hat, y_hat, theta_hat])


	
	valid_landmarks = []
	for i in range(len(sens.readings)):
		if numpy.sqrt((x_hat - sens.readings[i].landmark.x)**2 + (y_hat - sens.readings[i].landmark.y)**2) >= 0.1:
			valid_landmarks.append(sens.readings[i])
	valid_landmarks = numpy.array(valid_landmarks)

	

	F = numpy.zeros((3,3))
	F[0][0] = 1
	F[1][1] = 1
	F[2][2] = 1
	F[0][2] = -1 * self.step_size * sens.vel_trans * numpy.sin(self.x[2])
	F[1][2] = self.step_size * sens.vel_trans * numpy.cos(self.x[2])
	
	FT = numpy.transpose(F)


	P_hat = numpy.linalg.multi_dot([F, self.P, numpy.transpose(F)]) + self.V


	H = numpy.zeros((2*len(valid_landmarks), 3))
	for i in range (len(valid_landmarks)):
		H[2*i][0] = (x_hat - valid_landmarks[i].landmark.x) / (numpy.sqrt((x_hat - valid_landmarks[i].landmark.x)**2 + (y_hat - valid_landmarks[i].landmark.y)**2))
		H[2*i][1] = (y_hat - valid_landmarks[i].landmark.y) / (numpy.sqrt((x_hat - valid_landmarks[i].landmark.x)**2 + (y_hat - valid_landmarks[i].landmark.y)**2))
		H[2*i][2] = 0
		H[2*i+1][0] = (valid_landmarks[i].landmark.y - y_hat) / ((x_hat - valid_landmarks[i].landmark.x)**2 + (y_hat - valid_landmarks[i].landmark.y)**2)
		H[2*i+1][1] = (x_hat - valid_landmarks[i].landmark.x) / ((x_hat - valid_landmarks[i].landmark.x)**2 + (y_hat - valid_landmarks[i].landmark.y)**2)
		H[2*i+1][2] = -1

	HT = numpy.transpose(H)


	W = numpy.zeros((2*len(valid_landmarks), 2*len(valid_landmarks)))
	for i in range(len(valid_landmarks)):
		W[2*i][2*i] = 0.1
		W[2*i+1][2*i+1] = 0.05


	y = numpy.zeros((2*len(valid_landmarks), 1))
	for i in range(len(valid_landmarks)):
		y[2*i][0] = valid_landmarks[i].range
		y[2*i+1][0] = valid_landmarks[i].bearing


	h = numpy.zeros((2*len(valid_landmarks), 1))
	for i in range(len(valid_landmarks)):
		h[2*i][0] = numpy.sqrt((x_hat - valid_landmarks[i].landmark.x)**2 + (y_hat - valid_landmarks[i].landmark.y)**2)
		h[2*i+1][0] = math.atan2(valid_landmarks[i].landmark.y - y_hat, valid_landmarks[i].landmark.x - x_hat) - theta_hat

	
	S = numpy.linalg.multi_dot((H, P_hat, HT)) + W

	SI = numpy.linalg.inv(S)


	R = numpy.linalg.multi_dot((P_hat, HT, SI))

	
	nu = y - h


	for i in range(len(nu)):
		if nu[i] > numpy.pi and i%2 !=0:
			nu[i] = nu[i] - 2*numpy.pi
		elif nu[i] < -1*numpy.pi and i%2 != 0:
			nu[i] = nu[i] + 2*numpy.pi


	
	#last step (update)
	
	x_new = X_hat + numpy.dot(R, nu)

	self.x[0] = x_new[0]
	self.x[1] = x_new[1]
	self.x[2] = x_new[2]
 
	self.P = P_hat - numpy.linalg.multi_dot((R, H, P_hat))


	
        #### ----- YOUR CODE GOES HERE ----- ####
    
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0]
        est_msg.pose.y = self.x[1]
        est_msg.pose.theta = self.x[2]
        self.pub_est.publish(est_msg)

if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()
