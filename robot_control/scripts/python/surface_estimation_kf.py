#!/usr/bin/env python

# Author: Lasitha Wijayarathne
# This class is customized to match the functionality of the application.
import roslib; roslib.load_manifest('kuka_motion_lib')
import rospy

from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import PointStamped, WrenchStamped

import math
from numpy.random import randn

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

from numpy import dot
from scipy.linalg import inv

from iiwa_msgs.msg import JointPosition, StateEstimate
from kuka_kinematics import FK_kuka
from scipy.linalg import expm, sinm, cosm

import threading
import transformations as tr


# format the book
# import book_format
# book_format.set_style()

# import kf_book.book_plots as book_plots

global ft_lp, robot_position, KD, F, dt
dt             = 0.005
F              = np.zeros((5,5))
robot_position = np.eye(4)
robot_velocity = np.array([0.,0.,0.])
KD             = PointStamped()
ft_lp          = PointStamped()
robot_position_prev  = np.eye(4)
robot_pos_com        = np.eye(4)
robot_pos_com_prev   = np.eye(4)
robot_velocity_com   = np.array([0.,0.,0.])


def callback_ft(data):
    global ft_lp
    ft_lp.point.x = data.wrench.force.x
    ft_lp.point.y = data.wrench.force.y
    ft_lp.point.z = data.wrench.force.z

def callback_commanded_pos(data):
    global robot_pos_com, robot_pos_com_prev, robot_velocity_com
    robot_pos_com_prev = robot_pos_com
    robot_pos_com      = FK_kuka(data.position.quantity)    
    robot_velocity_com[2] = (robot_pos_com[2,3] - robot_pos_com_prev[2,3])/0.003
    print(robot_velocity_com[2])

def callback_robot_position(data):
    global robot_position, robot_position_prev, robot_velocity

    robot_position_prev  = robot_position
    robot_position       = FK_kuka(data.position.quantity)
    # robot_position[2,3] -= robot_position[2,3]
    # robot_position[1,3] -= robot_position[1,3]
    # robot_position[0,3] -= robot_position[0,3]
    robot_velocity[2]    = (robot_position[2,3] - robot_position_prev[2,3])/0.003
    print(robot_velocity[2])

def callback_KD(data):
    global KD, F, dt
    F[0,1] =  KD.point.z * dt
    F[0,2] = -KD.point.z * dt
    # rospy.loginfo('Contact parameters updated...')
    KD = data

def callback_init():
    # Subscribers
    rospy.Subscriber('/kuka/state/KUKAJointPosition', JointPosition, callback_robot_position) 
    rospy.Subscriber('/kuka/state/KUKAJointPositionCommand', JointPosition, callback_commanded_pos) 

    rospy.Subscriber('/ATI/filtered_lp', WrenchStamped, callback_ft) 
    # rospy.Subscriber('/netft_data', WrenchStamped, callback_ft) 

    rospy.Subscriber('/stiffness_id/KD', PointStamped, callback_KD) 
    
if __name__ == '__main__':

    rospy.init_node('kf_surface_movement')

    callback_init()

    # thread_ = threading.Thread(target=callback_init, args=())
    # thread_.start()

    rospy.loginfo('Subscribed to all Topics...')
    rate = rospy.Rate(1/dt)

    # Publishers
    # pub_est = rospy.Publisher('/environment_states/estimates', StateEstimate,  queue_size = 1)
    pub_est = rospy.Publisher('/environment_states/estimates', PointStamped,  queue_size = 1)
    pub_sf_normal = rospy.Publisher('/environment_states/surface_normal', PointStamped,  queue_size = 1)

    # esimated state for the surface
    x_est   = StateEstimate()
    x_est   = PointStamped()

    # surface normal of the force (from the low pass signal)
    sfn_est_n = PointStamped()
    n_m       = 0
    sfn_est   = np.array([[]])

    # kalman filter parameters
    # R_var = np.array([0.02, 0.0002])
    # Q_var = np.array([1e-8, 0, 2e-5, 0, 0, 6.99e-5])
    # Q_var = np.array([0.000, 0.000, 0.0001, 0.000, 0.000, 0.0001])
    # R_var = np.array([0.0002, 0002, 0.1, 0.1])

    Q_var = np.array([1e-10, 1e-10, 1e-10, 1e-8])
    R_var = np.array([0.0002, 0.001])

    # sleeping to wait for all the subscribers to initialize.
    # rospy.sleep(1)

    # x = np.array([ft_lp.point.z, 0., 0., robot_position[2,3], 0., 0.]).T
    x = np.array([ft_lp.point.z, 0., 0., 0.]).T

    print(x)

    # P = 10000000 * np.ones((6,6))
    P = 1000000 * np.eye(4)
    print(KD.point.x)

    # # kf model parameters
    # F = np.array([[0, 0, KD.point.z, 0, -KD.point.z, 0],
    #               [0, 0, 1, 0, 0, 0],
    #               [0, 0, 0, 0, 0, 0],
    #               [0, 0, 0, 0, 1, 0],
    #               [0, 0, 0, 0, 0, 1],
    #               [0, 0, 0, 0, 0, 0]])

    F = np.array([[0., 0., KD.point.z, -20],
                  [0., 0., 1., 0.],
                  [0., 0., 0., 1.],
                  [0., 0., 0., 0.]])


    # H = np.array([[1., 0.,0.],
    #               [0., 0., 0., 1., 0., 0.],
    #               [0., 0., 1., 0., 0., 0.],
    #               [0., 0., 0., 0., 1., 0.]])

    H = np.array([[1., 0., 0.,0.],
                  [0., 0., 1.,0.]])

    R = np.diagflat(np.power(R_var, 2))
    # R = Q_discrete_white_noise(dim=2, dt=dt, var=R_var)
    Q = np.diagflat(np.power(Q_var, 2))
    # Q = Q_discrete_white_noise(dim=5, dt=dt, var=Q_var)

    xs, cov = [], []

    F = expm(F * dt)
    gamma = 0.9325
    t_f_k = tr.rotation_matrix(gamma, [0, 0, 1])
    x_ = 0
    
    while not rospy.is_shutdown():

        # average surface normal force vector computation -----------------
        sfn_est = np.array([[ft_lp.point.x], [ft_lp.point.y], [ft_lp.point.z]]) 
        ## convert force sensor readings to kuka base frame
        stn_est = t_f_k[0:3,0:3] * sfn_est
        n_m     = np.sqrt(np.sum(sfn_est ** 2))
        if (n_m < 0.1):
            n_m = 1

        sfn_est_n.point.x = sfn_est[0] / n_m
        sfn_est_n.point.y = sfn_est[1] / n_m
        sfn_est_n.point.z = sfn_est[2] / n_m

        sfn_est_n.header.stamp = rospy.Time.now()
        pub_sf_normal.publish(sfn_est_n)
        # -----------------------------------------------------------------

        # predict
        x = dot(F, x)
        P = dot(F, P).dot(F.T) + Q

        # update
        S = dot(H, P).dot(H.T) + R
        K = dot(P, H.T).dot(inv(S))

        # treating as a linear 1D spring. project the end-effector in the direction of the average force normal. consider force tranformation to kuka frame
        proj_  = sfn_est * stn_est.T 
        deltaZ = np.matmul(proj_, robot_position[[[0],[1],[2]],[3]])
        
        
        # observation update
        y = np.array([ft_lp.point.z, robot_velocity[2]]) - dot(H, x) 
        # print(y)

        x += dot(K, y)
        P = P - dot(K, H).dot(P)

        # to make sure P is symmetric
        P = (P + P.T) / 2.0

        # publish the state estimate
        # print(P)

        x_ = x_ + 200*x[2]*dt

        x_est.point.x  = -x[1] 
        x_est.point.y  = robot_velocity_com[2]  # x[2]
        x_est.point.z  = robot_velocity[2] 
        # robot_velocity[2] - robot_velocity_com[2]

        x_est.header.stamp = rospy.Time.now()

        pub_est.publish(x_est)
        xs.append(x)
        cov.append(P)
        rate.sleep()



    xs, cov = np.array(xs), np.array(cov)
    # plot_track(xs[:, 0], track, zs, cov, plot_P=False, dt=dt)







