#!/usr/bin/env python

# Author: Lasitha Wijayarathne
# This class is customized to match the functionality of the application.

import roslib; roslib.load_manifest('kuka_motion_lib')
import rospy
from std_msgs.msg import String
import numpy as np
# from serial_ros.msg import customCmdMsg
from geometry_msgs.msg import PointStamped, WrenchStamped
from serial_ros.srv import *
from scipy import interpolate
from numpy.linalg import inv
from numpy.linalg import LinAlgError
from scipy.linalg import expm, sinm, cosm

import threading

# Classes to estimate the stiffness online using recursive least squares.
global cam_pos, ft_lp, ft_lp_

ft_lp = WrenchStamped()
ft_lp_ = WrenchStamped()
cam_pos = PointStamped()


def callback_ft_lp(data):
	global ft_lp_
	ft_lp_ = data

def callback_ft(data):
	global ft_lp
	ft_lp = data


def callback_cam(data):
	global cam_pos
	cam_pos = data


class rls:

	def __init__(self):
		# self.P_k    = 150 * np.matrix([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]), previous
		self.P_k    = 10 * np.matrix([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
		self.lmda   = 1.0
		self.R      = 0.0
		self.theta  = np.matrix([[1.0],[1.0],[1.0]])
		self.phi    = np.matrix([[10.0],[10.0],[10.0]])
		self.L_k    = np.matrix([[0.0],[0.0],[0.0]])

		self.F_h = np.zeros((1,3))
		self.x_h = np.zeros((1,3))
		self.init = False
		self.count_init = 0
		self.pub_ = rospy.Publisher('/test/ftc', PointStamped,  queue_size = 1)
		self.test = PointStamped()
		self.g = 1


	def rls_update(self, y_k, phi):

		phi  = phi * 0.0004
		y_k = -y_k

		self.test.point.x = phi * 1000
		self.test.point.y = y_k


		if self.init:
			
			y = self.F_h[0,2] + 2*self.F_h[0,1] + self.F_h[0,0]


			# self.L_k   = self.P_k * self.phi / (self.lmda + self.phi.T*self.P_k*self.phi)

			# self.theta = self.theta + self.L_k * (y - self.phi.T * self.theta)

			# self.P_k   = (1./self.lmda) * (self.P_k - self.L_k*self.phi.T*self.P_k)
			# self.P_k   = (self.P_k - self.L_k*self.phi.T*self.P_k)



			# self.L_k   = self.P_k * self.phi / (self.lmda + self.phi.T*self.P_k*self.phi)
			P_k_  = inv(inv(self.P_k) + self.phi*self.phi.T) 

			self.theta = self.theta + P_k_ * self.phi * (y - self.phi.T * self.theta)
			print(self.g)
			self.P_k   = 0.001 * P_k_ + self.g * np.eye(3)

			print(self.P_k)

			e = self.get_error(y, self.phi)

			# if (np.abs(e) > 0.35):
			# 	self.P_k = 150 * np.matrix([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])

			self.test.point.z = e
			# self.lmda_update(e)
			self.g_update(e)

			self.test.header.stamp = rospy.Time.now()
			self.pub_.publish(self.test)

			# get the error
			# print(e)
			# print(self.lmda)

			self.update_Fh(y_k)
			self.update_phi(phi)
		else:
			if (self.count_init==3):
				self.init = True
			else:
				self.update_Fh(y_k)
				self.update_phi(phi)
				self.count_init += 1
			
	def update_phi(self,phi):
		self.phi[2,0] = self.phi[1,0]
		self.phi[1,0] = self.phi[0,0]
		self.phi[0,0] = phi

	def update_Fh(self,y_k):
		self.F_h[0,0] = self.F_h[0,1]
		self.F_h[0,1] = self.F_h[0,2]
		self.F_h[0,2] = y_k

		# print(self.P_k)

	def get_theta(self):
		return self.theta

	def get_error(self, y_k, phi_):
		return (y_k - phi_.T * self.theta)

	# run this in a different thread
	def lmda_update(self, err):
		# self.lmda -= 1000*np.abs(err[0,0])
		# if (self.lmda > 0.6):
		# 	self.lmda = 1.0
		# elif(self.lmda < 0.1):
		# 	self.lmda = 0.3
		# print(self.lmda)
		self.lmda = 0.4 + (1-np.tanh(15*err[0,0])) * 0.6

	def g_update(self, err):
		self.g = 0.0 + (np.tanh(20*np.abs(err[0,0]))) * 3000000
		# self.g = 0.0 + (np.tanh(20*np.abs(err[0,0]))) * 100000

		# self.g = 0.1 +  (100 * np.abs(err[0,0])) * 1000000/3.5
		# self.g = 1000000 * np.exp(2*np.abs(err[0,0]))




# This class takes a batch of data and updates the contact parameters
class batch_ls:

	def __init__(self):
		self.theta  = np.array([[150, 0.02]])
		# self.phi    = np.matrix([[0,0]])

	def ls_update(self,y_k_N):
		#  generate and A and b matrices
		# xnew = np.arange(0, 9, 0.1)
		A,b = self.generate_matrices(y_k_N)
		try:
			self.theta = np.matmul(np.matmul(inv(np.matmul(A.T,A)), A.T),b) 
		except LinAlgError:
			rospy.loginfo("Matrix is singular")
		# print(np.matmul(np.matmul(inv(np.matmul(A.T,A)), A.T),b))

	def get_theta(self):
		return self.theta.T

	# this function syncronizes data and generates A and b matrices
	def generate_matrices(self, data):

		data_ = np.array(data[:])
		N = np.shape(data_)[0]

		A = np.zeros((N,2))
		b = np.zeros((N,1))

		time_cam = np.zeros((N,1))
		time_ft = np.zeros((N,1))

		cam_data = np.array((data_[:,0],))
		ft_data  = np.array((data_[:,1],))


		for i in range (0,N):
			time_cam[i,:] = cam_data[0,i].header.stamp.to_sec()
			time_ft[i,:]  = ft_data[0,i].header.stamp.to_sec()


			A[i,:] = [0.001*cam_data[0,i].point.x, np.sin(np.arccos(10*0.001*(cam_data[0,i].point.x)))]
			b[i,:] = ft_data[0,i].wrench.force.z

		# interpolate function
		cam_f = interpolate.interp1d(time_cam[:,0], A[:,0])
		ft_f  = interpolate.interp1d(time_ft[:,0], b[:,0])

		# start and end times
		time_min = np.amin([time_cam[-1,:], time_ft[-1,:]]) 
		time_max = np.amax([time_cam[0,:], time_ft[0,:]]) 

		# new time stamps
		t_new = np.arange(time_max, time_min, 0.005)
		cam_new = cam_f(t_new); #cam_new = (cam_new,)
		ft_new  = ft_f(t_new);  #ft_new  = (t_new,)

		N_new  = np.shape(cam_new)[0]
		# print(N_new)

		# A = np.resize(A,(N_new,2))
		# b = np.resize(b,(N_new,1))


		# b[:,0] = np.array(ft_new).T
		# A[:,0] = np.array(cam_new).T
		# A[:,1] = np.array(np.sin(np.arccos(4*(cam_new-0.25)))).T
		

		return A, b

# function method to call the service
def set_CAM_pwm_client(x, y):
    rospy.wait_for_service('set_cam_pwm')
    try:
        set_cam_pwm = rospy.ServiceProxy('set_cam_pwm', setCAMpwm)
        resp1 = set_cam_pwm(x, y)

        return resp1.active
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return 0

def on_shut_down():
	set_CAM_pwm_client(0.0, 0.0)
	rospy.sleep(1)
	rospy.loginfo("Motors switch off...")
	rospy.signal_shutdown('shutdown requested...')


if __name__ == '__main__':


	KD_params = PointStamped()
	# _history  = deque([]1

	rospy.init_node('stiffness_identification', disable_signals=True)

	# Subscribers
	rospy.Subscriber('/stiffness_probe/cam_vel', PointStamped, callback_cam) 
	rospy.Subscriber('ATI/filtered_bp', WrenchStamped, callback_ft) 

	# Publishers
	pub_kd = rospy.Publisher('/stiffness_id/KD', PointStamped,  queue_size = 1)
	ft_data   = [] # np.zeros((1000,3))

	# define the estimation classes
	rls_  = rls()
	bls_  = batch_ls()


	rate = rospy.Rate(500)
	rate_inner = rospy.Rate(120)
	time_start = 0.0
	# save  data to a finite size matrix
	# update the 
	surf_vel = 0.0
	set_CAM_pwm_client(0.0, surf_vel)

	# rospy.on_shutdown(on_shut_down)

	while not rospy.is_shutdown():

		try:
			# obtain the most recent measurement thorugh callbacks

			# calculate the stiffness, damping using rls. calculate only after cam is activated. call the service for activation
			# call the service. check if it is in contact first

			# calculate the stiffness, damping using batch_ls once a new batch is created.
			# wait for a new batch to be created

			time_start = rospy.Time.now().to_sec()
			# set_CAM_pwm_client(0.3, surf_vel)
			# i = 0; ft_data = []

			# while (time_start + 7.0 > rospy.Time.now().to_sec()):
			# 	#  calculte the stiffness, damping using rls.
			# 	if ((i < 1000) and (time_start + 1.5< rospy.Time.now().to_sec()) and (time_start + 6.0 > rospy.Time.now().to_sec())):
			# 		# appending data to the storage list
			# 		ft_data.append((cam_pos, ft_lp))

			# 		# update recursive lest squares estimation
			# 		rls_.rls_update(ft_lp.wrench.force.z, cam_pos.point.x)
			# 		i = i + 1

			# 	rate_inner.sleep()

			# recursive least squares estimation
			rls_.rls_update(ft_lp.wrench.force.z, cam_pos.point.x)
			# lease squares estimation
			# bls_.ls_update(ft_data)
			# set_CAM_pwm_client(0.0, surf_vel)

			# publish the current calculated parameter values
			temp_KD_rls = rls_.get_theta()
			# temp_KD_ls  = bls_.get_theta()

			# print(temp_KD_rls)

			KD_params.point.x = 1./4.* (6.2500e-06/4) * (temp_KD_rls[0,0] +  temp_KD_rls[2,0] -  temp_KD_rls[1,0])
			KD_params.point.y = (0.0025/2) * (temp_KD_rls[0,0] -  temp_KD_rls[1,0])
			KD_params.point.z = 0.6*(1./4.)*(temp_KD_rls[2,0] +  temp_KD_rls[1,0] +  temp_KD_rls[0,0])

			# for safety
			if (KD_params.point.z < 300.0):
				KD_params.point.z = 300;
			elif(KD_params.point.z > 7000):
				KD_params.point.z = 7000;

			print(KD_params)
			# print()

			KD_params.header.stamp = rospy.Time.now()

			pub_kd.publish(KD_params)
			rate.sleep()

		except KeyboardInterrupt:
			on_shut_down()

