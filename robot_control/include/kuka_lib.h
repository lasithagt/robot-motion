#ifndef KUKA_LIB_H
#define KUKA_LIB_H

#include <kdl/jntarray.hpp>


#include <cmath>
#include <memory>
#include <array>
#include <vector>

namespace joint_space_vel_prof {

   class velocityProfile {

   public:

      virtual void solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration){};
      virtual void get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const {};
      virtual void get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const {};
      virtual void get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const {};

      virtual ~velocityProfile(){}

   };



   class quinticPolynomial : public velocityProfile {
   private:
      
      bool use_vel_limits, use_acc_limits;
      double duration;
      std::vector<double> tf, tfv, tfa;
      std::vector<double> max_vel, max_acc;
      std::vector<double> p0, p3, p4, p5;
      std::vector<double> v2, v3, v4;
      std::vector<double> a1, a2, a3;
      KDL::JntArray qf_qi;

   public:

      quinticPolynomial(int N_joints, double m_vel[]);

      void solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration);
      void get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const;
      void get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const;
      void get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const;

      ~quinticPolynomial(){}
   };

   
   class cubicPolynomial : public velocityProfile {
   private:
      
      bool use_vel_limits, use_acc_limits;
      double duration;
      double N_joints;
      std::vector<double> tfv, tfa;
      std::vector<double> max_vel, max_acc;
      std::vector<double> p0, p2, p3;
      std::vector<double> v1, v2;
      std::vector<double> a0, a1;
      KDL::JntArray qf_qi;

   public:

      cubicPolynomial(int N_joints, double m_vel[]);

      std::vector<double> tf;
      virtual void solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration);
      virtual void get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const;
      virtual void get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const;
      virtual void get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const;

      virtual ~cubicPolynomial(){}
   };

   class trapezoidal : public velocityProfile {
   private:
      
      double duration;
      std::vector<double> max_vel, max_acc, switch_time, tf;
      KDL::JntArray qi_, qf_, qf_qi;

   public:

      trapezoidal(int N_joints);

      virtual void solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration);
      virtual void get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const;
      virtual void get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const;
      virtual void get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const;

      virtual ~trapezoidal(){}
   };
   
   
   bool getJointSpaceVelProfile(std::string vel_prof_name, std::unique_ptr<joint_space_vel_prof::velocityProfile>& vel_prof, double m_vel[]);
}
   
#endif