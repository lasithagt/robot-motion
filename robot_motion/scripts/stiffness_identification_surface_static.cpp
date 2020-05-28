
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

// define global variables
bool init = false;
bool init_time_bool = false;
bool init_f = false;
double f_thresh = -2;
bool sim = false;

Eigen::Vector3d f_data;
Eigen::Vector3d kd_data;
Eigen::Vector3d f_data_prev;
geometry_msgs::WrenchStamped netft_;
geometry_msgs::PointStamped switch_iter;
robot_interface::robotABSTRACT* robot_;
extern ros::Subscriber state_netft;


double myvfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
  if (!grad.empty()) {
    grad[0] = 0.0;
    grad[1] = 0.5 / sqrt(x[1]);
  }
  return sqrt(x[1]);
}

typedef struct {
    double a, b;
} my_constraint_data;

double myvconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
  my_constraint_data *d = reinterpret_cast<my_constraint_data*>(data);
  double a = d->a, b = d->b;
  if (!grad.empty()) {
    grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
    grad[1] = -1.0;
  }
  return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
}


void getFT(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
	init_f = true;
	f_data(0) = msg->wrench.force.x;
	f_data(1) = msg->wrench.force.y;
	f_data(2) = msg->wrench.force.z;
	ROS_INFO_STREAM("F_read");
	ROS_INFO_STREAM(f_data);
}

void getKD(const geometry_msgs::PointStamped::ConstPtr& msg) {
	kd_data(0) = msg->point.x;
	kd_data(1) = msg->point.y;
	kd_data(2) = msg->point.z;


}


void read_data(int &n_data_q, std::unique_ptr<Eigen::MatrixXf> &data_q) {

   	ROS_INFO_STREAM("Starting reading the trajectory file."); 
    n_data_q = read_txt_files::readFile_nlines(getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/desired_position_FM_dynamic.txt"));

    ROS_INFO_STREAM(n_data_q);

    data_q.reset(new Eigen::MatrixXf);
    data_q.get()->resize(KUKA_N_JOINTS, n_data_q); 

    int ret = read_txt_files::readFile(*(data_q.get()), getenv("HOME") + std::string("/Documents/ROS_ws/src/kuka-motion/data/desired_position_FM_dynamic.txt"));
    ROS_INFO_STREAM("Finished reading the trajectory file.");
}


bool term_cond_3(double time) {
	if (time < 0.5) {
		return false;
	}
	return true;
}


int main (int argc, char** argv) {


	const std::string node_name = "kuka_FM";
	ros::init(argc, argv, node_name);
   	ros::NodeHandle nh_states("~");
   	ros::NodeHandle nh_command("~");

   	// Read from file. Make to read from the paramserver 
   	int n_data_q;
    std::unique_ptr<Eigen::MatrixXf> data_q;
    
    read_data(n_data_q, data_q);
    Eigen::MatrixXf* data_ = data_q.get();
   
	// Publishers 
	ros::Publisher robot_motion_ready  = nh_command.advertise<std_msgs::Bool>("/kuka/command/active", 1);
	// ros::Publisher netft_pub		   = nh_states.advertise<geometry_msgs::WrenchStamped>("/netft/state", 1);
	ros::Publisher pub_switch_dir	   = nh_states.advertise<geometry_msgs::PointStamped>("/state/switch", 1);
	

	// Subscribers
	if (!sim) {
		// ros::Subscriber state_netft        = nh_states.subscribe("/netft_data", 1, getFT);
		robot_ = new robot_interface::robotKUKA();
		// robot_ = (robot_interface::robotKUKA)robot;
	} else {
		// ros::Subscriber state_netft        = nh_states.subscribe("/iiwa/ft_sensor_ee", 1, getFT);
		robot_ = new robot_interface::robotKUKA_SIM();
		// robot_interface::robotKUKA_SIM robot_;
	}
	ros::Subscriber state_netft = nh_states.subscribe("/netft_data", 1, getFT);
	ros::Subscriber state_stiffness = nh_states.subscribe("/stiffness_id/KD", 1, getKD);
	// ros::Subscriber state_netft = nh_states.subscribe("/ATI/filtered_lp", 1, getFT);

	
	std::string vel_prof_name = "cubic_polynomial";

	const double rate_interp = 500.0;
	ros::Rate rate_(rate_interp);

	// create an array of evaluating points. These are points at which splines should be evaluated.
	int n_linspace = 36;
	int* eval_array = utils::linspace(10, n_data_q, n_linspace);

	// joint values
	double q[7]; 
	int term_FM = 0;
	double hz = 500; double dt = 1 / hz;
	int F_contact = 0;
	double vel_O = 0.001; // Linear velocity when doing orientation control

	// change the direction of movement.
	bool dir_1 = true; bool dir_2 = false; bool wait = true; double wait_start_time = 0.0; int dir_iter = 1; 
	int f_count = 0; int track_pos = 0;
	
	// define the robot class, this includes FWK and Jacobian functions
	robot_def::robot kuka_; 


	robot_->init(nh_states, nh_command);

	ROS_INFO_STREAM("Initialized Robot...");

	iiwa_msgs::JointPosition position;
	position.position.quantity.resize(7);

	iiwa_msgs::JointPosition position_init;
	position_init.position.quantity.resize(7);

	iiwa_msgs::JointPosition position_cmd;
	position_cmd.position.quantity.resize(7);

	double duration = 15.0;
	double duration_init = 20.0;
	KDL::JntArray q_final(7); 
	q_final.data(0) = (*data_)(0,f_count); q_final.data(1) = (*data_)(1,f_count); q_final.data(2) = (*data_)(2,f_count); q_final.data(3) = (*data_)(3,f_count); q_final.data(4) = (*data_)(4,f_count); 
	q_final.data(5) = (*data_)(5,f_count); q_final.data(6) = (*data_)(6,f_count);
    

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

    robot_motion_primitives::robot_motion robot_position = robot_motion_primitives::robot_motion(*robot_, kuka_, 7, 600, vel_prof_name);
    // robot_position.move_joint_to(q_final, position, duration_init);

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

    double dt_o = 0.00001;
    double dt_p = dt * 1.0/1000.0;
    double f_d = 5;



	if (robot_->getRobotIsConnected()) {
		q_final.data(0) = (*data_)(0,f_count); q_final.data(1) = (*data_)(1,f_count); q_final.data(2) = (*data_)(2,f_count); q_final.data(3) = (*data_)(3,f_count); q_final.data(4) = (*data_)(4,f_count); 
		q_final.data(5) = (*data_)(5,f_count); q_final.data(6) = (*data_)(6,f_count);

		// // set these temporally
		// q_final.data(0) = 0; q_final.data(1) = 0.8727; q_final.data(2) = 0; q_final.data(3) = 0.8727*2; q_final.data(4) = 0; 
		// q_final.data(5) = 0.8727; q_final.data(6) = 0;

		robot_position.move_joint_to(q_final, position_init, duration_init);
		ROS_INFO_STREAM("Moving to initial point...");

		position_init.position.quantity[0] = (*data_)(0,f_count);
		position_init.position.quantity[1] = (*data_)(1,f_count);
		position_init.position.quantity[2] = (*data_)(2,f_count);
		position_init.position.quantity[3] = (*data_)(3,f_count);
		position_init.position.quantity[4] = (*data_)(4,f_count);
		position_init.position.quantity[5] = (*data_)(5,f_count);
		position_init.position.quantity[6] = (*data_)(6,f_count);


    ROS_INFO_STREAM("Completed initial positioning...");
    // compute the orientation error

    Eigen::Vector3d axis_x(1,0,0);
    Eigen::Vector3d axis_z(0,0,1);
    Eigen::Matrix3d temp;
    if (!sim) {
    	temp = Eigen::AngleAxisd(0.9325, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    } else {
    	temp = Eigen::Matrix3d::Identity();
    }

    Eigen::Vector3d f_data_transformed;
    robot_->getJointPosition(position);
    position_cmd = position;
    position_init = position;

    // integral error
    Eigen::Vector3d err_integral(0.0,0.0,0.0);
    const double K_I = 10;


    while (ros::ok() || !ros::isShuttingDown()) {

    		// dt_p = dt * 1.0 / kd_data(2);
	    	memcpy(q, position_cmd.position.quantity.data(), 7*sizeof(double));
			kuka_.fwk(fwk_m, q);

			Eigen::Vector3d f_test;
			f_test(0) = 0; f_test(1) = 0.0; f_test(2) = 1;

			Eigen::Vector3d v = fwk_m.block(0,0,3,3) * v_ref;

			// ROS_INFO_STREAM(v);
			f_data_transformed = fwk_m.block(0,0,3,3) * temp.transpose() * f_data;

			ROS_INFO_STREAM("force_transformed");
			ROS_INFO_STREAM(f_data_transformed);

			// check if the ee is in contact
			if (f_data_transformed.norm() < 0.1 || f_data_transformed.norm() > 10) {
				q_err = Eigen::Quaterniond::FromTwoVectors(v, f_test); 
			} else {
				q_err = Eigen::Quaterniond::FromTwoVectors(v, f_data_transformed); 
			}

			// compute the positional error
			Eigen::Vector3d f_data_normalized = f_data_transformed.normalized();
			

			p_err = -f_d*f_data_normalized + f_data_transformed;

			// integrating the error
			err_integral += p_err * dt;

			if (err_integral.norm() > 1.3 ) {
				err_integral(0) = 0.0; err_integral(1) = 0.0; err_integral(2) = 0.0;
			}

			// concatanate positional and orientation error
			// p_err(0) *= err_integral(0); p_err(1) *= err_integral(1); p_err(2) *= err_integral(2);
			dp << p_err  * dt_p, q_err.vec() * dt_o;

			ROS_INFO_STREAM(q_err.vec());
			ROS_INFO_STREAM("positional error...");
			ROS_INFO_STREAM(p_err);

			// compute the jacobian inverse (generalized) and make use of the null space
			kuka_.jacobian(Jac, q); 
			Eigen::VectorXd dq = Jac.completeOrthogonalDecomposition().solve(dp);

			// ROS_INFO_STREAM(dq);

			for (int i=0;i < 7;i++) {
				if (std::abs(dq[i]) > 0.5) {
					dq[0] = 0.0; dq[1] = 0.0; dq[2] = 0.0; dq[3] = 0.0; dq[4] = 0.0;  dq[5] = 0.0; dq[6] = 0.0; 
					ROS_INFO_STREAM("beyond limits...");

				}
			}
			// update the next pose
			//
			for (int i=0;i < 7;i++) {
				position_cmd.position.quantity[i] = position_cmd.position.quantity[i] + dq[i] + dt*(position_init.position.quantity[i]-position_cmd.position.quantity[i]);
				// position_cmd.position.quantity[i] = position_cmd.position.quantity[i] + 0.001*(position_init.position.quantity[i]-position_cmd.position.quantity[i]);

			}

			// Set the next position
			robot_->setJointPosition(position_cmd);

			// Eigen::Quaterniond rotatedP = q * p * q.inverse(); 

			// robot_position.move_constant_orientation(position, qdot, vel_O, term_cond) ;
			rate_.sleep();

		}
		// ros::shutdown();
		
	}

	command_active.data = false;
    robot_motion_ready.publish(command_active);

	ROS_INFO_STREAM("Shutting down...");

	ros::shutdown();
	

	return 0;

}





