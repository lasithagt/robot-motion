#ifndef ROS_KUKA_MODEL_H
#define ROS_KUKA_MODEL_H


#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <memory>
#include <string.h>
#include <iostream>
#include <algorithm>

#include "models.h"
#include "robot_abstract.h"

using namespace KDL;

namespace ros_kuka 
{

struct KUKAModelKDLInternalData : RobotAbstractInternalData
{
    int numJoints;
    Eigen::Matrix<double, 4, 4> baseTransform;
    Eigen::MatrixXd Kv; // joint dynamic coefficient
    Eigen::MatrixXd Kp;

    Eigen::Matrix<double, 6, 6> jacTransform;

    KUKAModelKDLInternalData() = default;
    void init()
    {
        jacTransform.setZero();
        auto R = baseTransform.block(0,0,3,3);
        jacTransform.block(0,0,3,3) = R;
        jacTransform.block(3,3,3,3) = R;
    }
};

class KUKAModelKDL : public RobotAbstract
{

public:
    KUKAModelKDL(const Chain& robotChain, const KUKAModelKDLInternalData& robotParams);
    ~KUKAModelKDL();
    int initRobot();
    void getForwardKinematics(double* q, double* qd, double *qdd, Eigen::Matrix<double,3,3>& poseM, Eigen::Vector3d& poseP, Eigen::Vector3d& vel, Eigen::Vector3d& accel, bool computeOther);

    /* given q, qdot, qddot, outputs torque output*/
    void getInverseDynamics(double* q, double* qd, double* qdd, Eigen::VectorXd& torque);
    void getForwardDynamics(double* q, double* qd, const Eigen::VectorXd& force_ext, Eigen::VectorXd& qdd);
    void getMassMatrix(double* q, Eigen::MatrixXd& massMatrix);
    void getCoriolisMatrix(double* q, double* qd, Eigen::VectorXd& coriolis); // change
    void getGravityVector(double* q, Eigen::VectorXd& gravityTorque);
    void getSpatialJacobian(double* q, Eigen::MatrixXd& jacobian);
    void getSpatialJacobianDot(double* q, double* qd, Eigen::MatrixXd& jacobianDot);
    void ik();

    KUKAModelKDLInternalData robotParams_;

public:
    // patch variables for speed
    JntArray q_;   
    JntArray qd_;   
    JntArray qdd_;
    JntSpaceInertiaMatrix inertia_mat_;     // Interia Matrix
    JntArray coriolis_;                     // CoriolisVector
    JntArray gravity_;                      // GravityVector
    KDL::Jacobian jacobian_;
    Frame frame_;
    FrameVel frame_vel_;

    Eigen::Matrix<double, 7, 7> Kv_;
    ChainDynParam* dynamicsChain_;
    Chain robotChain_;

};

}

#endif // KUKA_MODEL_HPP
