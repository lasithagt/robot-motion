
#include <robot_kinematics.hpp> 

/* TODO */

namespace robot_def {

robot::robot() {

}

// robot::~robot() {

// }

void robot::fwk_internal(double* FK, const double* q ) {
//
  FK[0] = ((((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*cos(q[5]) + ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*sin(q[5]))*cos(q[6]) + (-((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*sin(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*cos(q[4]))*sin(q[6]);
  FK[1] = -((((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*cos(q[5]) + ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*sin(q[5]))*sin(q[6]) + (-((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*sin(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*cos(q[4]))*cos(q[6]);
  FK[2] = -(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]);
  FK[3] = -0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]) + 0.4*(-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - 0.4*sin(q[1])*cos(q[0])*cos(q[3]) - 0.42*sin(q[1])*cos(q[0]);
  FK[4] = ((((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*cos(q[5]) + ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*sin(q[5]))*cos(q[6]) + (-((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*cos(q[4]))*sin(q[6]);
  FK[5] = -((((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*cos(q[5]) + ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*sin(q[5]))*sin(q[6]) + (-((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*cos(q[4]))*cos(q[6]);
  FK[6] = -(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]);
  FK[7] = -0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]) + 0.4*(sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - 0.4*sin(q[0])*sin(q[1])*cos(q[3]) - 0.42*sin(q[0])*sin(q[1]);
  FK[8] = (((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*cos(q[5]) + (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*sin(q[5]))*cos(q[6]) + (-(sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*sin(q[2])*cos(q[4]))*sin(q[6]);
  FK[9] = -(((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*cos(q[5]) + (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*sin(q[5]))*sin(q[6]) + (-(sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[4]) - sin(q[1])*sin(q[2])*cos(q[4]))*cos(q[6]);
  FK[10] = -((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]);
  FK[11] = -0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]) + 0.4*sin(q[1])*sin(q[3])*cos(q[2]) + 0.4*cos(q[1])*cos(q[3]) + 0.42*cos(q[1]) + 0.36;
  FK[12] = 0;
  FK[13] = 0;
  FK[14] = 0;
  FK[15] = 1;

}



void robot::jacobian_internal(double* jac, const double* q ) {

  jac[0] = 0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) - 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]) - 0.4*(sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) + 0.4*sin(q[0])*sin(q[1])*cos(q[3]) + 0.42*sin(q[0])*sin(q[1]);
  jac[1] = -(-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]) + 0.4*sin(q[1])*sin(q[3])*cos(q[2]) + 0.4*cos(q[1])*cos(q[3]) + 0.42*cos(q[1]))*cos(q[0]);
  jac[2] = -(-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]) + 0.4*(sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - 0.4*sin(q[0])*sin(q[1])*cos(q[3]) - 0.42*sin(q[0])*sin(q[1]))*cos(q[1]) - (-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]) + 0.4*sin(q[1])*sin(q[3])*cos(q[2]) + 0.4*cos(q[1])*cos(q[3]) + 0.42*cos(q[1]))*sin(q[0])*sin(q[1]);
  jac[3] = (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*(-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]) + 0.4*sin(q[1])*sin(q[3])*cos(q[2]) + 0.4*cos(q[1])*cos(q[3])) + (-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]) + 0.4*(sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - 0.4*sin(q[0])*sin(q[1])*cos(q[3]))*sin(q[1])*sin(q[2]);
  jac[4] = ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*(-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]) + 0.4*sin(q[1])*sin(q[3])*cos(q[2]) + 0.4*cos(q[1])*cos(q[3])) - (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*(-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]) + 0.4*(sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - 0.4*sin(q[0])*sin(q[1])*cos(q[3]));
  jac[5] = -(-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]))*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[4])) + (((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[4]) - (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*cos(q[4]))*(-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]));
  jac[6] = (-(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]))*(-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5])) - (-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]))*(-((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]));
  jac[7] = -0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]) + 0.4*(-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - 0.4*sin(q[1])*cos(q[0])*cos(q[3]) - 0.42*sin(q[1])*cos(q[0]);
  jac[8] = -(-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]) + 0.4*sin(q[1])*sin(q[3])*cos(q[2]) + 0.4*cos(q[1])*cos(q[3]) + 0.42*cos(q[1]))*sin(q[0]);
  jac[9] = (-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]) + 0.4*(-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - 0.4*sin(q[1])*cos(q[0])*cos(q[3]) - 0.42*sin(q[1])*cos(q[0]))*cos(q[1]) + (-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]) + 0.4*sin(q[1])*sin(q[3])*cos(q[2]) + 0.4*cos(q[1])*cos(q[3]) + 0.42*cos(q[1]))*sin(q[1])*cos(q[0]);
  jac[10] = -(-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*(-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]) + 0.4*sin(q[1])*sin(q[3])*cos(q[2]) + 0.4*cos(q[1])*cos(q[3])) - (-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]) + 0.4*(-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - 0.4*sin(q[1])*cos(q[0])*cos(q[3]))*sin(q[1])*sin(q[2]);
  jac[11] = -((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*(-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]) + 0.4*sin(q[1])*sin(q[3])*cos(q[2]) + 0.4*cos(q[1])*cos(q[3])) + (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*(-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]) + 0.4*(-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - 0.4*sin(q[1])*cos(q[0])*cos(q[3]));
  jac[12] = (-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]))*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[4])) - (((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*sin(q[4]) - (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*cos(q[4]))*(-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]));
  jac[13] = -(-(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]))*(-0.257*((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + 0.257*(sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5])) + (-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]))*(-((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]));
  jac[14] = 0;
  jac[15] = (-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]) + 0.4*(-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - 0.4*sin(q[1])*cos(q[0])*cos(q[3]) - 0.42*sin(q[1])*cos(q[0]))*cos(q[0]) + (-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]) + 0.4*(sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - 0.4*sin(q[0])*sin(q[1])*cos(q[3]) - 0.42*sin(q[0])*sin(q[1]))*sin(q[0]);
  jac[16] = (-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]) + 0.4*(-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - 0.4*sin(q[1])*cos(q[0])*cos(q[3]) - 0.42*sin(q[1])*cos(q[0]))*sin(q[0])*sin(q[1]) - (-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]) + 0.4*(sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - 0.4*sin(q[0])*sin(q[1])*cos(q[3]) - 0.42*sin(q[0])*sin(q[1]))*sin(q[1])*cos(q[0]);
  jac[17] = (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*(-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]) + 0.4*(sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - 0.4*sin(q[0])*sin(q[1])*cos(q[3])) - (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*(-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]) + 0.4*(-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - 0.4*sin(q[1])*cos(q[0])*cos(q[3]));
  jac[18] = ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*(-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]) + 0.4*(sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - 0.4*sin(q[0])*sin(q[1])*cos(q[3])) - ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*(-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]) + 0.4*(-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - 0.4*sin(q[1])*cos(q[0])*cos(q[3]));
  jac[19] = -(-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]))*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[4]) - (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*cos(q[4])) + (-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]))*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*sin(q[4]) - (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*cos(q[4]));
  jac[20] = (-(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]))*(-0.257*(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + 0.257*((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5])) - (-0.257*(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + 0.257*((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]))*(-(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]));
  jac[21] = 0;
  jac[22] = sin(q[0]);
  jac[23] = -sin(q[1])*cos(q[0]);
  jac[24] = -sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]);
  jac[25] = (-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]);
  jac[26] = ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*sin(q[4]) - (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*cos(q[4]);
  jac[27] = -(((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*cos(q[3]) + sin(q[1])*sin(q[3])*cos(q[0]))*cos(q[4]) + (-sin(q[0])*cos(q[2]) - sin(q[2])*cos(q[0])*cos(q[1]))*sin(q[4]))*sin(q[5]) + ((-sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))*sin(q[3]) - sin(q[1])*cos(q[0])*cos(q[3]))*cos(q[5]);
  jac[28] = 0;
  jac[29] = -cos(q[0]);
  jac[30] = -sin(q[0])*sin(q[1]);
  jac[31] = -sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]);
  jac[32] = (sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]);
  jac[33] = ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*sin(q[4]) - (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*cos(q[4]);
  jac[34] = -(((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*cos(q[3]) + sin(q[0])*sin(q[1])*sin(q[3]))*cos(q[4]) + (-sin(q[0])*sin(q[2])*cos(q[1]) + cos(q[0])*cos(q[2]))*sin(q[4]))*sin(q[5]) + ((sin(q[0])*cos(q[1])*cos(q[2]) + sin(q[2])*cos(q[0]))*sin(q[3]) - sin(q[0])*sin(q[1])*cos(q[3]))*cos(q[5]);
  jac[35] = 1;
  jac[36] = 0;
  jac[37] = cos(q[1]);
  jac[38] = -sin(q[1])*sin(q[2]);
  jac[39] = sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]);
  jac[40] = (sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*sin(q[4]) + sin(q[1])*sin(q[2])*cos(q[4]);
  jac[41] = -((sin(q[1])*cos(q[2])*cos(q[3]) - sin(q[3])*cos(q[1]))*cos(q[4]) - sin(q[1])*sin(q[2])*sin(q[4]))*sin(q[5]) + (sin(q[1])*sin(q[3])*cos(q[2]) + cos(q[1])*cos(q[3]))*cos(q[5]);
  
}

// returns the jacobian with a Eigen Matrix type
void robot::fwk(Eigen::MatrixXd &FWK, const double* q) {
  
  fwk_internal(fwk_, q);
  FWK = Eigen::Map<Eigen::MatrixXd>(fwk_, 4, 4);
  FWK.transposeInPlace();

}

// returns the jacobian with a Eigen Matrix type
void robot::jacobian(Eigen::MatrixXd &Jac, const double* q) {
  
  jacobian_internal(jac, q);
  Jac = Eigen::Map<Eigen::MatrixXd>(jac, 7, 6);
  Jac.transposeInPlace();

}

void robot::jacobian_null(double* jac, const double* q) { 

  // Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
  // Eigen::MatrixXd A_null_space = lu.kernel();
}

void robot::jacobian_null_O(Eigen::MatrixXd &Jac, Eigen::MatrixXd &Jac_null_space, const double* q) { 

  // get the jacobian 
  jacobian(Jac,q);
  Jac_null_space = Jac.block(3,0,3,7).fullPivLu().kernel();

}

// void 



}