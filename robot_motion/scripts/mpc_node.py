#!/usr/bin/env python

# Author: Lasitha Wijayarathne
# This class is customized to match the functionality of the application.
import roslib; roslib.load_manifest('kuka_motion_lib')
import rospy
from numpy import cos, sin, pi, absolute, arange, array, matmul
from scipy.signal import kaiserord, lfilter, firwin, freqz
import numpy as np
from pylab import figure, clf, plot, xlabel, ylabel, xlim, ylim, title, grid, axes, show
from geometry_msgs.msg import WrenchStamped, PointStamped
from std_msgs.msg import Time
import matplotlib.pyplot as plt
from iiwa_msgs.msg import JointPosition, StateEstimate
from kuka_kinematics import FK_kuka

import sys

sys.path.append("../src/Resources") 
from MPCprob import QP_MPC

global QP, est_state, ft_lp, robot_position, sfn_normal
robot_position 	  	= np.zeros((4,4))
ft_lp             	= PointStamped()
est_state 	     	= StateEstimate()
est_state.quantity 	= [0,0,0,0,0,0,0]
sfn_normal        	= PointStamped()

# callback to get the avergae surface force normal
def callback_sfn(data):
	global sfn_normal
	sfn_normal = data

# callback to get the position of the KUKA
def callback_robot_position(data):
    global robot_position
    robot_position = FK_kuka(data.position.quantity)
    # print(robot_position)


# callback to get the lowpass FT data
def callback_ft_lp(data):
	global ft_lp
	ft_lp.point.x = data.wrench.force.x
	ft_lp.point.y = data.wrench.force.y
	ft_lp.point.z = data.wrench.force.z


def callback_state_estimate(data):
    global est_state
    est_state = data

# to be implemented to compute the coefiicients in a online manner
def online_time_series_model():
	# data = pd.read_csv('https://vincentarelbundock.github.io/Rdatasets/csv/datasets/sunspot.year.csv')

	# data.index = data['time'].values

	# plt.figure(figsize=(15,5))
	# plt.plot(data.index,data['sunspot.year'])
	# plt.ylabel('Sunspots')
	# plt.title('Yearly Sunspot Data');

	# model = pf.ARIMA(data=data, ar=4, ma=4, target='sunspot.year', family=pf.Normal())
	# x = model.fit("MLE")
	return 0


def mpc_control_seq(x, ref, plot_=False):
	U = QP.mpc_rtn(x, ref, plot_)
	return U

def init_MPC():
	global QP
	A = np.zeros((16,16))
	K = 250
	deltaT = 0.01

	A[0,:] = np.array([1,0,0,0,0,0,0,0,0,0,0,K*deltaT, 0, 0, -K*deltaT,0])
	A[1:11, 2:12] = np.eye(10)
	A[-1-4,-1-4] = 1
	A[-1-4,-1-3] = deltaT
	A[-1-3, 1:11] = np.array([-3.2554, 3.3643,-0.5260,-0.8991, 0.3114,-0.2502, 0.1744, 0.4726, -0.5620,  0.1701])

	A[-1-2,:] = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,1,deltaT,0.001])
	A[-1-1,:] = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,deltaT])

	C   = np.array([[1],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]).T
	B   = np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[1]])
	d   = []
	Q   = np.array([[1]])
	R   = np.array([[6.9]])
	QN  = Q

	# m = 1
	Ulb = np.array([[-4.0]])
	Uub = np.array([[4.0]])

	# n = 16
	Xlb = -1 * 5 * np.ones((1,16))  
	Xub =  1 * 5 * np.ones((1,16))  

	#-------------------------------------------------------------------------------------------------------------------#

	yr   = np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]])
	ulin = []
	qlin = []
	N    = 10

	# print(quadprog_solve_qp(P, q, G, h))
	x0  = np.array([[0.0],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.]])
	QP = QP_MPC(A, B, C, d, Q, R, QN, N, Ulb, Uub, Xlb, Xub, x0, yr, ulin, qlin)

    # QP.mpc_rtn(x0, np.array([[-1]]))

if __name__ == '__main__':

	dp = PointStamped()
	rospy.init_node('qp_mpc')
	pub_dp = rospy.Publisher('/MPC/command/', PointStamped, queue_size = 1)

	# subscribers
	# estimated states
	rospy.Subscriber("/environment_states/estimates", StateEstimate, callback_state_estimate) 
	rospy.Subscriber("/environment_states/surface_normal", StateEstimate, callback_sfn) 

	rate_pub = rospy.Rate(100)

	# initialize MPC
	init_MPC()	

	x0  = np.array([[0.0],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.],[0.]])
	ref = np.array([[-3]])

	while not rospy.is_shutdown():
    	
		# get the current state estimation
		x0 = np.array([est_state.quantity[0], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., est_state.quantity[1], est_state.quantity[2], est_state.quantity[3], est_state.quantity[4], est_state.quantity[5]])

		# get the optimal u_next
		# [U, X]  = mpc_control_seq(x0, ref, plot_=False)

		# # publish the optimal next X, assuming X is column wise
		# dp.point.z = X[-1-2, 0]
		# pub_dp.publish(dp)

		rate_pub.sleep()






