/**
 * @file robot_class.hpp
 *
 * @brief This message displayed in Doxygen Files index
 *
 * @ingroup PackageName
 * (Note: this needs exactly one @defgroup somewhere)
 *
 * @author Lasitha Wijayarathne
 * Contact: lasitha@gatech.edu
 *
 */

#ifndef ROBOT_MOTION_LIBRARY_H
#define ROBOT_MOTION_LIBRARY_H

#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>

#include <ecl/geometry.hpp>
#include <ecl/containers.hpp>
#include <ecl/exceptions.hpp>
#include <ecl/errors.hpp>
#include <ecl/concepts.hpp>
#include <ecl/converters.hpp>


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <Eigen/QR> 
#include <vector>
#include <robot_kinematics.hpp>
#include "robot_interface/robot_class.hpp"
#include "robot_abstract.h"
#include "kuka_lib.h"
#include <memory>
#include "interpolation.h" // alglib

// write this as a class
namespace robot_motion_primitives {

    class robot_motion {

    public:
        robot_motion(robot_interface::robotABSTRACT &robot_,  const std::shared_ptr<RobotAbstract>& robot_d, int N_JOINTS, double rate, const std::string polynomial_type);
        ~robot_motion();

        bool move_joint_to(KDL::JntArray &q_final, iiwa_msgs::JointPosition &position_, double& duration);
        bool move_joint_to_wrench(KDL::JntArray &q_final, iiwa_msgs::JointPosition &position_, double& duration, bool wrench);
        void move_joint_velocity();
        void check_delta_q();
        void move_null();
        void move_null_position();
        void move_null_orientation(Eigen::MatrixXd& FK, Eigen::MatrixXd& Jac, Eigen::MatrixXd &Jac_null_O, const double* q, Eigen::VectorXd &qdot, double vel);
        void move_constant_orientation(iiwa_msgs::JointPosition &position_, Eigen::VectorXd &qdot, double vel, bool(&term_c)(double time_start));

    
        std::vector<ecl::CubicSpline> splines;
        std::vector<alglib::spline1dinterpolant> splines_pp_;
        robot_interface::robotABSTRACT* robot_interface_;
        std::shared_ptr<RobotAbstract> robot_def;

        joint_space_vel_prof::velocityProfile* vel_prof;
        const double publish_rate;
        const int N_JOINTS;
        double curr_time;

    };

}

#endif