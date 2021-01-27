
/**
 * @file robot_motion_library.cpp
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
#include <iostream>

#include <robot_motion_library.hpp>

namespace robot_motion_primitives {

    /** constructor for the motion library class
    *
    * If no verbosity is specified, inherit from parent
    *
    * @param name component of interest
    * @param verbosity desired trace verbosity
    */
    robot_motion::robot_motion(robot_interface::robotABSTRACT &robot_, const std::shared_ptr<RobotAbstract>& robot_d, int N_JOINTS_, double rate, 
        const std::string polynomial_type) : N_JOINTS(N_JOINTS_), publish_rate(rate), robot_def(robot_d)
    {

        // define the robot interface (sim or real)
        robot_interface_ = &robot_;
        robot_interface_->getRobotTime(curr_time);
        ROS_INFO_STREAM(curr_time);

        // define robot kinematics and dynamic parameters
        // robot_def_ = &robot_d;

        // reserve the number of splines in vector
        splines.resize(N_JOINTS);
        splines_pp_.resize(N_JOINTS);

        // quintic spline parameters

        // load parameters from the parameters server.
        double m_vel[] = {1, 1, 1, 1, 1, 1, 1};
        vel_prof = new joint_space_vel_prof::quinticPolynomial(N_JOINTS, m_vel);
        
        curr_time = 0.0;
        

    }

    /** destructor for the motion library class
    *
    * If no verbosity is specified, inherit from parent
    *
    * @param name component of interest
    * @param verbosity desired trace verbosity
    */
    robot_motion::~robot_motion() {
        delete(vel_prof);
    }



    /** Quintic interpolation of motion from point to point given the time duration (gazebo time for simulations and KRC time for kuka simulations)
    *
    * If no verbosity is specified, inherit from parent
    *
    * @param name component of interest
    * @param verbosity desired trace verbosity
    */
    bool robot_motion::move_joint_to(KDL::JntArray &q_final, iiwa_msgs::JointPosition &position_, double& duration) {

        KDL::JntArray q_curr(7);
        robot_interface_->getJointPosition(position_);
        ROS_INFO_STREAM(position_);
        ros::Rate rate_interp(publish_rate);

        // double curr_time = 0.0;

        // get current time from the robot
        robot_interface_->getRobotTime(curr_time);
        const double init_time = curr_time;
        
        double new_time = curr_time - init_time;

        memcpy(q_curr.data.data(), position_.position.quantity.data(),  7*sizeof(double));
        vel_prof->solve(q_curr, q_final, duration);

        // TODO: exception catch
        while (new_time < duration && ros::ok && !ros::isShuttingDown()) {
            robot_interface_->getRobotTime(curr_time);
            new_time = curr_time - init_time;
            
            vel_prof->get_desired_joint_pos(q_curr, new_time);

            // check if robot object is initialized first. for command_position --> shared pointer
            memcpy(position_.position.quantity.data(), q_curr.data.data(), 7*sizeof(double));
            robot_interface_->setJointPosition(position_);

            rate_interp.sleep();
        }

        return true;
    }

    /** Quintic interpolation of motion from point to point given the time duration (gazebo time for simulations and KRC time for kuka simulations)
    *
    * If no verbosity is specified, inherit from parent
    *
    * @param name component of interest
    * @param verbosity desired trace verbosity
    */
    bool robot_motion::move_joint_to_wrench(KDL::JntArray &q_final, iiwa_msgs::JointPosition &position_, double& duration, bool wrench) {

        KDL::JntArray q_curr(7);
        robot_interface_->getJointPosition(position_);
        ROS_INFO_STREAM(position_);
        ros::Rate rate_interp(publish_rate);

        // double curr_time = 0.0;
        iiwa_msgs::Wrench wrench_ref;
        wrench_ref.wrench.quantity.resize(7); // resize the wrench quantity

        // get current time from the robot
        robot_interface_->getRobotTime(curr_time);
        const double init_time = curr_time;
        
        double new_time = curr_time - init_time;

        memcpy(q_curr.data.data(), position_.position.quantity.data(),  7*sizeof(double));
        vel_prof->solve(q_curr, q_final, duration);

        // TODO: exception catch
        while (new_time < duration && ros::ok && !ros::isShuttingDown()) {
            robot_interface_->getRobotTime(curr_time);
            new_time = curr_time - init_time;
            
            vel_prof->get_desired_joint_pos(q_curr, new_time);

            // check if robot object is initialized first. for command_position --> shared pointer
            memcpy(position_.position.quantity.data(), q_curr.data.data(), 7*sizeof(double));
            robot_interface_->setJointPosition(position_);


            if (wrench) {
                wrench_ref.wrench.quantity[0] = 0;
                wrench_ref.wrench.quantity[1] = 0;
                wrench_ref.wrench.quantity[2] = 1;

                robot_interface_->setWrench(wrench_ref);
            }

            rate_interp.sleep();
        }

        return true;
    }


    // Moves the joint with the given the velocity. Move it to a control interface class along with position and torque controls
    /** Set a component's verbosity.
    *
    * If no verbosity is specified, inherit from parent
    *
    * @param name component of interest
    * @param verbosity desired trace verbosity
    */
    void robot_motion::move_joint_velocity() {}

    // Checks if the current pose is far away from the commanded. Sets the command_active inactive
    void robot_motion::check_delta_q() {}    

    // TODO
    /** Set a component's verbosity.
    *
    * If no verbosity is specified, inherit from parent
    *
    * @param name component of interest
    * @param verbosity desired trace verbosity
    */
    void robot_motion::move_null() {}

    // TODO
    /** Set a component's verbosity.
    *
    * If no verbosity is specified, inherit from parent
    *
    * @param name component of interest
    * @param verbosity desired trace verbosity
    */
    void robot_motion::move_null_position() {}

    /** Set a component's verbosity.
    *
    * If no verbosity is specified, inherit from parent
    *
    * @param name component of interest
    * @param verbosity desired trace verbosity
    */
    void robot_motion::move_null_orientation(Eigen::MatrixXd& FK, Eigen::MatrixXd& Jac, Eigen::MatrixXd &Jac_null_O, const double* q, Eigen::VectorXd &qdot, double vel) {

        Eigen::Vector3d b(0, 0, vel); // Orientation vector z axis

        // robot_def_->jacobian_null_O(Jac, Jac_null_O, q);
        // robot_def_->fwk(FK, q);

        qdot = Jac_null_O * ((Jac.block(0,0,3,7)*Jac_null_O).completeOrthogonalDecomposition().solve(FK.block(0,0,3,3)* b));

    }

    /** moves with a constant orientation given the termination condition
    *
    * If no verbosity is specified, inherit from parent
    *
    * @param name component of interest
    * @param verbosity desired trace verbosity
    */
    void robot_motion::move_constant_orientation(iiwa_msgs::JointPosition &position_, Eigen::VectorXd &qdot, double vel, bool(&term_c)(double time_start)) {

        const double dt = 0.002;
        Eigen::MatrixXd Jac_null_O(3,4);
        Eigen::MatrixXd Jac(6,7);
        Eigen::MatrixXd FK(4,4);
        ros::Rate rate_(500);
        double q[7];

        Eigen::VectorXd temp_(7);
        // Eigen::VectorXd qdot(7);

        double curr_time = 0.0;

        // get current time from the robot
        robot_interface_->getRobotTime(curr_time);
        const double init_time = curr_time;
        double new_time = curr_time - init_time;

        // get the current position
        robot_interface_->getJointPosition(position_);

        while (!ros::isShuttingDown() && ros::ok && !term_c(new_time)) {
            // ROS_INFO_STREAM("Here in while");

            memcpy(q, position_.position.quantity.data(), 7*sizeof(double));

            // compute qdot
            move_null_orientation(FK, Jac, Jac_null_O, q, qdot, vel);

            memcpy(temp_.data(), position_.position.quantity.data(), 7*sizeof(double));
            // update the position (compute next position)
            temp_ = temp_ + qdot * dt; 
     
            memcpy(position_.position.quantity.data(), temp_.data(), 7*sizeof(double));

            // set the position
            robot_interface_->setJointPosition(position_);
            robot_interface_->getRobotTime(curr_time);
            new_time = curr_time - init_time;
            rate_.sleep();
        }

    }
}

    /*
    void spline_interp(std::vector<ecl::CubicSpline> &splines, Eigen::MatrixXf &data, const int n_data, double start_time, double stop_time);
    void spline_piecewise_interp(std::vector<alglib::spline1dinterpolant> &splines, Eigen::MatrixXf &data, double start_time, double end_time);


    }
    */
