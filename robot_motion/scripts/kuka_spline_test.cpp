
// Example code for kuka torque control.

#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string.h> 
#include "kuka_lib.h"

#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <robot_kinematics.hpp>
#include <kuka_motion_library.hpp>


#include <memory>
#include <boost/program_options.hpp>
#include <exception>

#include <utils.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif



using namespace joint_space_vel_prof;

int main (int argc, char** argv) {

	const std::string node_name = "kuka_move_path";
	ros::init(argc, argv, node_name);
   	ros::NodeHandle nh("~");
   	
   	float pub_rate_hz;

   	// Read from file. Make to read from the paramserver
   	int n_data_qd, n_data_q;
   	ROS_INFO_STREAM("Starting reading the trajectory file."); 
    n_data_q = read_txt_files::readFile_nlines(getenv("HOME") + std::string("/Documents/ROS_ws/kuka_ws/src/kuka_motion_lib/data/desired_position_2.txt"));


    ROS_INFO_STREAM(n_data_q);
    Eigen::MatrixXf data_q(7, n_data_q);


    int ret = read_txt_files::readFile(data_q, getenv("HOME") + std::string("/Documents/ROS_ws/kuka_ws/src/kuka_motion_lib/data/desired_position_2.txt"));
    ROS_INFO_STREAM("Finished reading the trajectory file.");
    // -----------------------------------------------------------------------------------------------------------------------------------------
    ROS_INFO_STREAM(data_q.rows());

    
	ros::Rate rate(1);
	double curr_time = 0.0;
	std::vector<ecl::CubicSpline> splines;
	std::vector<ecl::CubicSpline> splines_;
	splines.resize(7);
	splines_.resize(7);
	const int n_data = n_data_q;
	double start_time = 0.0;
	double stop_time = 10.0;
	double command_pos = 0.0;

	spline_traj::spline_interp(splines, data_q, n_data, start_time, stop_time);



	// spline interpolation
	std::vector<alglib::spline1dinterpolant> splines_p;
	splines_p.resize(7);

	// spline interpolation for direction 2
	Eigen::MatrixXf data_q2(7, n_data_q);
	// data_q2 = data_q;
	// data_q2 = data_q2.colwise().reverse();


	for (int i = 0;i < n_data; i++) {
		data_q2.col(i) = data_q.col(n_data-1-i);
	}

	for (int i = 0;i < n_data; i++) {
		data_q.col(i) = data_q2.col(n_data-1-i);
	}


	spline_traj::spline_interp(splines_, data_q2, n_data, start_time, stop_time);

	std::cout << data_q2.col(0).transpose() << std::endl;
	std::cout << data_q2.col(n_data-1).transpose() << std::endl;
	std::cout << "\n" << std::endl;

	std::cout << data_q.col(0).transpose() << std::endl;
	std::cout << data_q.col(n_data-1).transpose() << std::endl;

	std::vector<alglib::spline1dinterpolant> splines_p2;
	splines_p2.resize(7);
	double start_time_2 = start_time;
	double stop_time_2 = stop_time;
	spline_traj::spline_piecewise_interp(splines_p, data_q, start_time, stop_time);
	spline_traj::spline_piecewise_interp(splines_p2, data_q2, start_time_2, stop_time_2);

	std::cout << "\n" << std::endl;
	spline_traj::spline_piecewise_interp(splines_p2, data_q2, start_time_2, stop_time_2);

	// build splines for each joint
	
	std::cout << "\n" << std::endl;
	std::cout << spline1dcalc(splines_p[0], stop_time) << " " << spline1dcalc(splines_p[1], stop_time) << " " << spline1dcalc(splines_p[2], stop_time) << " " << spline1dcalc(splines_p[3], stop_time) << " " << spline1dcalc(splines_p[4], stop_time) << " " << spline1dcalc(splines_p[5], stop_time) << std::endl;
	std::cout << spline1dcalc(splines_p2[0], start_time) << " " << spline1dcalc(splines_p2[1], start_time) << " " << spline1dcalc(splines_p2[2], start_time) << " " << spline1dcalc(splines_p2[3], start_time) << " " << spline1dcalc(splines_p2[4], start_time) << " " << spline1dcalc(splines_p2[5], start_time) << std::endl;

	
	int* eval_array = spline_traj::linspace(0, n_data_q, 36);

	std::cout << eval_array[35] << std::endl;
	

	//-------------------------------------//
	double *jac = new double[4];
	jac[0] = 1; jac[1] = 2; jac[2] = 3; jac[3] = 4;
	
	Eigen::MatrixXd test(2, 2);
	Eigen::MatrixXd eigenX = Eigen::Map<Eigen::MatrixXd>( jac, 2, 2);

	std::cout << eigenX << std::endl;

	Eigen::MatrixXf temp = data_q.block(0,2,7,50);
	spline_traj::spline_interp(splines, temp, temp.cols(), 0, 2);
	std::cout << "\n" << std::endl;
	std::cout << splines[3](1) << std::endl;
	std::cout << "\n" << std::endl;

	robot_def::robot kuka;
	const double q[7] = {0,0.7854,0,1.5708,0,0.7854,0};
	Eigen::VectorXd qdot(7);
	robot_motion_primitives::move_null_orientation(kuka, q, qdot,0.02);
	std::cout << qdot << std::endl;


	while(ros::ok()) {
		// Increment the time

		curr_time += 0.001;
		const double temp = splines[0].domain().back();
		try {
			if (curr_time < temp) {
				// command_pos = splines[3](curr_time);
				// std::cout << command_pos << std::endl;
				std::cout << spline1dcalc(splines_p[3], curr_time) << std::endl;
				std::cout << spline1dcalc(splines_p2[3], curr_time) << std::endl;
			}
		} catch (std::exception& e) {
			std::cout << "Error" << std::endl;
		}
		
		// ROS_INFO_STREAM(curr_time);

		ros::spinOnce();
		rate.sleep();

	}

	return 0;

	}





