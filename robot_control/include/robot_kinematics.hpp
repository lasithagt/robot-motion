/*
This defines kuka kinematics functions for the kuka iiwa R820 with the an endeffector (EndEffector code: 1)

*/

#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Core>
#include <vector>
#include <Eigen/LU>

/* write this as a class */
namespace robot_def {

class robotAbstractParams {

  public:
    virtual ~robotAbstractParams(){};
    virtual void fwk_internal(double*, const double*){};
    virtual void fwk(Eigen::MatrixXd &FWK, const double* q){};
    virtual void jacobian_internal(double*, const double*){};
    virtual void jacobian_null(double*, const double*){};
    virtual void jacobian_null_O(Eigen::MatrixXd& , Eigen::MatrixXd& , const double*){};
    virtual void jacobian(Eigen::MatrixXd& , const double* ){};
    virtual void inverse_dynamics(Eigen::MatrixXd&){};
    virtual void M_matrix(Eigen::MatrixXd&){};
    virtual void G_matrix(Eigen::VectorXd&){};
};


class robot : public robotAbstractParams {
  double jac[42]; // make this private
  double fwk_[16];

  public:
    robot();
    ~robot(){};
    void fwk_internal(double*, const double*);
    void fwk(Eigen::MatrixXd &FWK, const double* q);
    void jacobian_internal(double*, const double*);
    void jacobian_null(double*, const double*);
    void jacobian_null_O(Eigen::MatrixXd& , Eigen::MatrixXd& , const double* );
    void jacobian(Eigen::MatrixXd& , const double* );
};



}

#endif