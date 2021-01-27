
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string.h> 
#include <cmath>

#include <iostream>
#include <fstream>

#include <std_msgs/Time.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>

#include <robot_kinematics.hpp>
#include <robot_motion_library.hpp>
#include <robot_class.hpp>
#include <utils.hpp>
#include <kuka_lib.h>

#include <boost/program_options.hpp>

#include <algorithm>
#include <nlopt.hpp>
#include <iomanip>


#define KUKA_N_JOINTS 7 
#ifndef M_PI
#define M_PI 3.14159265358979
#endif


// TODO:
// 1. implement Torque Control Example.
// 2. implement Wrench Control Example. 

double f_thresh = -2;
bool sim = true;

Eigen::Vector3d f_data;
Eigen::Vector3d kd_data;
Eigen::Vector3d f_data_prev;
geometry_msgs::WrenchStamped netft_;
geometry_msgs::PointStamped switch_iter;
extern ros::Subscriber state_netft;


// callback for force vector
void getFT(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	init_f = true;
	f_data(0) = msg->wrench.force.x;
	f_data(1) = msg->wrench.force.y;
	f_data(2) = msg->wrench.force.z;
}

// get the updated environmental parameters
void getKD(const geometry_msgs::PointStamped::ConstPtr& msg) {
	kd_data(0) = msg->point.x;
	kd_data(1) = msg->point.y;
	kd_data(2) = msg->point.z;

}


int main (int argc, char** argv) {


	const std::string node_name = "kuka_admittance_control";
	ros::init(argc, argv, node_name);
   	ros::NodeHandle nh_states("~");
   	ros::NodeHandle nh_command("~");

   	// Read from file. Make to read from the paramserver TODO
   	int n_data_q;
    
    Eigen::MatrixXf* data_ = data_q.get();
   
	// Publishers 
	ros::Publisher robot_motion_ready  = nh_command.advertise<std_msgs::Bool>("/kuka/command/active", 1);
	// ros::Publisher netft_pub		   = nh_states.advertise<geometry_msgs::WrenchStamped>("/netft/state", 1);
	ros::Publisher pub_switch_dir	   = nh_states.advertise<geometry_msgs::PointStamped>("/state/switch", 1);
	
	robot_interface::robotABSTRACT* robot_interface;

	// Subscribers
	if (!sim) {
		// ros::Subscriber state_netft        = nh_states.subscribe("/netft_data", 1, getFT);
		robot_interface = new robot_interface::robotKUKA();
		// robot_ = (robot_interface::robotKUKA)robot;
	} else {
		// ros::Subscriber state_netft        = nh_states.subscribe("/iiwa/ft_sensor_ee", 1, getFT);
		robot_interface = new robot_interface::robotKUKA_SIM();
		// robot_interface::robotKUKA_SIM robot_;
	}

	// ros::Subscriber state_netft        = nh_states.subscribe("/iiwa/ft_sensor_ee", 1, getFT);
	// ros::Subscriber state_netft        = nh_states.subscribe("/netft_data", 1, getFT);
	ros::Subscriber state_netft     = nh_states.subscribe("/ATI/filtered_lp", 1, getFT);
	ros::Subscriber state_stiffness = nh_states.subscribe("/stiffness_id/KD", 1, getKD);

	
	std::string vel_prof_name = "cubic_polynomial";

	const double rate_interp = 500.0;
	ros::Rate rate_(rate_interp);


	// joint values
	double q[7]; 
	int term_FM = 0;
	double hz = 500; double dt = 1 / hz;
	int F_contact = 0;
	double vel_O = 0.006; // Linear velocity when doing orientation control

	/* ------------------------------------------------------ Initialize the robot ---------------------------------------------------- */

	KDL::Chain kuka = KDL::KukaDHKdl();

	KUKAModelKDLInternalData robotParams;
	robotParams.numJoints = 7;
	robotParams.Kv = Eigen::MatrixXd(7,7);
	robotParams.Kp = Eigen::MatrixXd(7,7);

	KUKAModelKDL* kukaRobot = new KUKAModelKDL(kuka, robotParams);
	KukaRobot->initRobot();


	double timeFromStart = 0;

	double contactForce[3] = {0,0,0};

	/* ------------------------------------------------------ Admittace Force Controller ---------------------------------------------------- */
	ForceControl::ContactParams cp_FC;
	cp_FC.K = 100;
	cp_FC.D = 10;
	double dt = 0.01;
	bool is_sim = true;

	// double* force_current = new double[3];
	Eigen::Vector3d force_current(0,0,0);
	Eigen::Vector3d force_desired(0,0,5);

	double gains[3] = {1,1,1};
	Eigen::VectorXd q_curr(7);
	q_curr << 0,0,0,0,0,0,0;
	Eigen::VectorXd update_q(7);
	Eigen::Vector3d position_ref(0,0,1.1); 
	Eigen::Vector3d orientation_ref(0,0,0);
	Eigen::Vector3d poseQ(0,0,0);


	Eigen::VectorXd q_desired(7);
	q_desired.setZero();

	ForceControl::AdmittanceForceController AdmittanceControl = ForceControl::AdmittanceForceController(cp_FC, dt);
	AdmittanceControl.update(*kukaRobot, q_curr, poseP, poseQ, q_desired, force_current, force_desired, gains, update_q);

	std::cout << update_q.transpose().format(CleanFmt) << std::endl;

    Eigen::Vector3d axis_x(1,0,0);
    Eigen::Vector3d axis_z(0,0,1);
    Eigen::Matrix3d trans_FT;


    Eigen::Vector3d f_data_transformed;

    /* ------------------------------------ Transformation from the force sensor frame to the EE frame -----------------------------------_*/
    if (!sim) {
    	trans_FT = Eigen::AngleAxisd(0.9325, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    } else {
    	trans_FT = Eigen::Matrix3d::Identity();
    }
	
	/* ---------------------------------------------- End - Admittace Force Controller --------------------------------------------------- */


	Eigen::VectorXd qdot(7);

	ROS_INFO_STREAM("Initializing the spinner...");
	// run the spinner
	ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

	
    ros::Duration(1.0).sleep();

    ROS_INFO_STREAM("Finished initializing the spinner...");
    ROS_INFO_STREAM(robot_->getRobotIsConnected());

    double curr_time = 0.0;

    robot_->getRobotTime(curr_time);
    ROS_INFO_STREAM(curr_time);
    std_msgs::Bool command_active;

    command_active.data = true;
    robot_motion_ready.publish(command_active);

    /* ------------------------------------------------ Initialize the robot for motion --------------------------------------------------------*/
    robot_motion_primitives::robot_motion robot_position = robot_motion_primitives::robot_motion(*robot_interface, 7, 600, vel_prof_name);

    bool static_pos = true;

    // variables for the force modulation
    Eigen::Vector3d v_ref(0,0,1);	
    Eigen::Quaterniond q_err;	
    Eigen::Vector3d p_err;
    Eigen::MatrixXd fwk_m(4,4);
    Eigen::MatrixXd Jac(6,7);
    Eigen::VectorXd dp(6);
    Eigen::VectorXd q_next(7);
    double q_f[7];

    double dt_o = 0.00000;
    double dt_p = dt * 0.0025;
    double f_d  = 5;

    /* ------------------------------------- TEST: Move the robot to an arbitrary point ------------------------------------- */ 
	if (robot_interface->getRobotIsConnected()) {
		q_final = data_.col(0);

		robot_position.move_joint_to(q_final, position_ref, duration_init);
		ROS_INFO_STREAM("Moving to initial point...");

	}


    ROS_INFO_STREAM("Completed initial positioning...");
    ROS_INFO_STREAM("Sleeping for 5 seconds...");
    ros::Duration(5.0).sleep();

    ROS_INFO_STREAM("Starting the Trajectory...");

    /* ------------------------------------- TEST: Get robot position ------------------------------------- */ 

    robot_interface->getJointPosition(position);
    position_cmd = position;
    position_ref = position;


	KDL::JntArray q_curr(7);

	curr_time = 0.0;

    /* ------------------------------------- TEST: Get robot position ------------------------------------- */ 
	robot_->getRobotTime(curr_time);
	const double init_time = curr_time;

	double new_time = curr_time - init_time;
	memcpy(q_curr.data.data(), position.position.quantity.data(),  7*sizeof(double));


    std::vector<ecl::CubicSpline> splines_;
    robot_position.move_spline_to_ret(*data_, position, splines_, duration);

    // integral error
    Eigen::Vector3d err_integral(0.0,0.0,0.0);
    double K_I = 10*1/20; //  double K_I = 16*1/20;

    // check if robot object is initialized first. for command_position --> shared pointer
    
    while (ros::ok() && !ros::isShuttingDown() && (new_time < splines_[0].domain().back()) && (new_time < duration)) {


    	// update the next pose. (next position)
        for (int i = 0; i < 7;i++) {
            q_curr(i) = splines_[i](new_time);
        }
        ROS_INFO_STREAM("Updating te reference trajectory");

        // commented out such it will not move
		memcpy(position_ref.position.quantity.data(), q_curr.data.data(), 7*sizeof(double));


		// calculate the fwk nd jac
    	memcpy(q, position_cmd.position.quantity.data(), 7*sizeof(double));

		Eigen::Vector3d f_test;
		f_test(0) = 0; f_test(1) = 0.0; f_test(2) = 1;

		Eigen::Vector3d v = fwk_m.block(0,0,3,3) * v_ref;

		// ROS_INFO_STREAM(v);
		f_data_transformed = fwk_m.block(0,0,3,3) * temp.transpose() * f_data;

		ROS_INFO_STREAM("force_transformed");
		ROS_INFO_STREAM(f_data_transformed);

		/* check if the ee is in contact */
		if (f_data_transformed.norm() < 0.1 || f_data_transformed.norm() > 10) {
			q_err = Eigen::Quaterniond::FromTwoVectors(v, f_test); 
		} else {
			q_err = Eigen::Quaterniond::FromTwoVectors(v, f_data_transformed); 
		}

		// compute the positional error
		Eigen::Vector3d f_data_normalized = f_data_transformed.normalized();
		p_err = -f_d*f_data_normalized + f_data_transformed;

		AdmittanceControl.update(*kukaRobot, q_curr, poseP, poseQ, q_desired, force_current, force_desired, gains, update_q);

		// concatanate positional and orientation error

		dp << p_err, q_err.vec() * dt_o;

		ROS_INFO_STREAM("positional error...");
		ROS_INFO_STREAM(p_err);


		if (b_limits == true) {
			ROS_INFO_STREAM("beyond limits...");
		}

		for (int i = 0;i < 7; i++) {
			if (std::abs(dq[i]) > 0.5) {

				dq[0] = 0.0; dq[1] = 0.0; dq[2] = 0.0; dq[3] = 0.0; dq[4] = 0.0;  dq[5] = 0.0; dq[6] = 0.0; 
				ROS_INFO_STREAM("beyond limits...");
    			b_limits = true;
			}
		}


		for (int i = 0;i < 7; i++) {
			position_cmd.position.quantity[i] = position_cmd.position.quantity[i] + dq[i] + dt * (position_ref.position.quantity[i] - position_cmd.position.quantity[i]);
		}

		// Set the next position
		robot_->setJointPosition(position_cmd);

    	// get the curr time
    	robot_->getRobotTime(curr_time);
		new_time = curr_time - init_time;

		
		rate_.sleep();

	}

	command_active.data = false;
    robot_motion_ready.publish(command_active);

	ROS_INFO_STREAM("Shutting down...");
	ros::shutdown();
	

	return 0;

}

