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
namespace robot_tree {

class robotTreeAbstract {

  public:
    virtual void fwk_internal(double*, const double*)=0;
    virtual void fwk(Eigen::MatrixXd &FWK, const double* q)=0;
    virtual void jacobian_internal(double*, const double*)=0;
    virtual void jacobian_null(double*, const double*)=0;
    virtual void jacobian_null_O(Eigen::MatrixXd& , Eigen::MatrixXd& , const double*)=0;
    virtual void jacobian(Eigen::MatrixXd& , const double* )=0;
    virtual void inverse_dynamics(Eigen::MatrixXd&)=0;
    virtual void M_matrix(Eigen::MatrixXd&)=0;
    virtual void G_matrix(Eigen::VectorXd&)=0;
};


class robot_dual : public robotTreeAbstract {
  double jac[42]; // make this private
  double fwk_[16];

  public:
    void fwk_internal(double*, const double*);
    void fwk(Eigen::MatrixXd &FWK, const double* q);
    void jacobian_internal(double*, const double*);
    void jacobian_null(double*, const double*);
    void jacobian_null_O(Eigen::MatrixXd& , Eigen::MatrixXd& , const double* );
    void jacobian(Eigen::MatrixXd& , const double* );
};



}

#endif