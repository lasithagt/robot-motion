#include "kuka_lib.h"
#include <iostream>

namespace joint_space_vel_prof {


cubicPolynomial::cubicPolynomial(int N_joints, double vel_m[]) {

   
   use_vel_limits = true;
   max_vel.resize(N_joints);
   for (int i = 0;i < N_joints; i++) {
      max_vel[i] = vel_m[i];
   }
   qf_qi.resize(N_joints);
   tf.resize(N_joints);
   tfv.resize(N_joints);
   tfa.resize(N_joints);
   p0.resize(N_joints);
   p2.resize(N_joints);
   p3.resize(N_joints);
   v1.resize(N_joints);
   v2.resize(N_joints);
   a0.resize(N_joints);
   a1.resize(N_joints);
}

//Find the trajectory duration for each joint.
void cubicPolynomial::solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration)
{ 
   KDL::Subtract(qf,qi,qf_qi);
   
   if(use_vel_limits && !use_acc_limits) {
      for(int i=0; i<max_vel.size(); ++i) {
         //if (qf_qi(i) == 0) {
            tf[i] = duration;
         //}

         //tf[i] = 3 * std::abs(qf_qi(i)) / (2 * max_vel[i]);
      }
   } 
   else if(!use_vel_limits && use_acc_limits) {
      for(int i=0; i<max_acc.size(); ++i) {
         tf[i] = std::sqrt((6 * std::abs(qf_qi(i))) / max_acc[i]);
      }
	}
   else if(use_vel_limits && use_acc_limits) {
      for(int i=0; i<max_vel.size(); ++i) {
         tfv[i] = (3 * (qf_qi(i))) / (2 * max_vel[i]);
         tfa[i] = std::sqrt((6 * (qf_qi(i))) / max_acc[i]);
         tfv[i] > tfa[i] ? tf[i] = tfv[i] : tf[i] = tfa[i];
      }
   }

   // coefficients 
   for(int i=0; i<qi.rows(); ++i){

      p0[i] = qi(i);
   // p1[i] = 0;
      p2[i] = 3 * qf_qi(i) / (tf[i] * tf[i]);
      p3[i] = -2 * qf_qi(i) / (tf[i] * tf[i] * tf[i]);
      
   // v0[i] = 0;
      v1[i] = 2 * p2[i];
      v2[i] = 3 * p3[i];
      
      a0[i] = v1[i];
      a1[i] = 2 * v2[i];
      // printf("%f\n",tf[i] );
   }


   
   duration = *std::max_element(tf.begin(), tf.end());

}

//Current desired joint positions.
void cubicPolynomial::get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_pos.rows(); ++i) {
      desired_joint_pos(i) = (p3[i] * current_trajec_time + p2[i]) * current_trajec_time * current_trajec_time + p0[i];
      
   }
}

//Current desired joint velocities.
void cubicPolynomial::get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_vel.rows(); ++i)
      desired_joint_vel(i) = current_trajec_time * (v2[i] * current_trajec_time + v1[i]);
}

//Current desired joint acceleration.
void cubicPolynomial::get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const 
{
   for(int i=0; i<desired_joint_acc.rows(); ++i)
      desired_joint_acc(i) = a1[i] * current_trajec_time + a0[i];
}

//Quintic polynomial velocity profile.
quinticPolynomial::quinticPolynomial(int N_joints, double vel_m[]) {


   use_vel_limits = true;
   qf_qi.resize(N_joints);

   max_vel.resize(N_joints);
   for (int i = 0;i < N_joints; i++) {
      max_vel[i] = vel_m[i];
   }

   tf.resize(N_joints);
   tfv.resize(N_joints);
   tfa.resize(N_joints);
   qf_qi.resize(N_joints);
   p0.resize(N_joints);
   p3.resize(N_joints);
   p4.resize(N_joints);
   p5.resize(N_joints);
   v2.resize(N_joints);
   v3.resize(N_joints);
   v4.resize(N_joints);
   a1.resize(N_joints);
   a2.resize(N_joints);
   a3.resize(N_joints);
}

//Find the trajectory duration for each joint.
void quinticPolynomial::solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration)
{
   KDL::Subtract(qf,qi,qf_qi);

   // if(use_vel_limits && !use_acc_limits) {
   //    for(int i=0; i<max_vel.size(); ++i) {
   //       tf[i] = (15 * std::abs(qf_qi(i))) / (8 * max_vel[i]);
   //    }
   // }   
   // else if(!use_vel_limits && use_acc_limits) {
   //    for(int i=0; i<max_acc.size(); ++i) {
   //       tf[i] = std::sqrt(10 * std::abs(qf_qi(i)) / (std::sqrt(3) * max_acc[i]));
   //    }
   // }
   // else if(use_vel_limits && use_acc_limits) {
   //    for(int i=0; i<max_vel.size(); ++i){
   //       tfv[i] = (15 * (qf_qi(i))) / (8 * max_vel[i]);
	  //     tfa[i] = std::sqrt((10 * (qf_qi(i))) / (std::sqrt(3) * max_acc[i]));
	  //     tfv[i] > tfa[i] ? tf[i] = tfv[i] : tf[i] = tfa[i];
   //    }
   // }

   if(use_vel_limits && !use_acc_limits) {
      for(int i=0; i<max_vel.size(); ++i) {
         //if (qf_qi(i) == 0) {
            tf[i] = duration;
         //}

         //tf[i] = 3 * std::abs(qf_qi(i)) / (2 * max_vel[i]);
      }
   } 
      
   // else RTT::log(RTT::Error) << "Can not create quintic polynomial velocity profile. \n"
	//			"Either \"use_joint_vel_lims\" or \"use_joint_acc_lims\" must be true." << RTT::endlog();
   
   for(int i=0; i<qi.rows(); ++i) {

      p0[i] = qi(i);
   // p1[i] = 0;
   // p2[i] = 0;
      p3[i] = 10 * qf_qi(i) / std::pow(tf[i],3);
      p4[i] = -15 * qf_qi(i) / std::pow(tf[i],4);
      p5[i] = 6 * qf_qi(i) / std::pow(tf[i],5);
   // v0[i] = 0;
   // v1[i] = 0;
      v2[i] = 3 * p3[i];
      v3[i] = 4 * p4[i];
      v4[i] = 5 * p5[i];
   // a0[i] = 0;
      a1[i] = 2 * v2[i];
      a2[i] = 3 * v3[i];
      a3[i] = 4 * v4[i]; 
   }
   
   duration = *std::max_element(tf.begin(), tf.end());
}

//Current desired joint positions.
void quinticPolynomial::get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_pos.rows(); ++i)
      desired_joint_pos(i) = (current_trajec_time * (p5[i] * current_trajec_time + p4[i]) + p3[i]) * std::pow(current_trajec_time, 3) + p0[i];
}

//Current desired joint velocities.
void quinticPolynomial::get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const
{
   for(int i=0; i<desired_joint_vel.rows(); ++i)
      desired_joint_vel(i) = current_trajec_time * current_trajec_time * (current_trajec_time * (p5[i] * current_trajec_time + p4[i]) + p3[i]);
}

//Current desired joint accelerations.
void quinticPolynomial::get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const 
{
   for(int i=0; i<desired_joint_acc.rows(); ++i)
      desired_joint_acc(i) = current_trajec_time * (current_trajec_time * (a3[i] * current_trajec_time + a2[i]) + a1[i]); 
}



//Trapezoidal velocity profile
trapezoidal::trapezoidal(int N_joints)
{

   printf("here");
   /*qi_.resize(N_joints);
   qf_.resize(N_joints);
   qf_qi.resize(N_joints);
   max_vel.resize(N_joints);
   max_acc.resize(N_joints);
   switch_time.resize(N_joints);
   tf.resize(N_joints);*/
}

//Find the trajectory duration for each joint.
void trapezoidal::solve(const KDL::JntArray& qi, const KDL::JntArray& qf, double& duration)
{
   for(int i=0; i<qf.rows(); ++i)
   {
      switch_time[i] = max_vel[i] / max_acc[i];
      
      qf_qi(i) = qf(i) - qi(i);
      
      tf[i] = switch_time[i] + qf_qi(i) / max_vel[i];
      
      qi_(i) = qi(i);
      
      qf_(i) = qf(i);
   }
   
   duration = *std::max_element(tf.begin(), tf.end());
}

//Current desired joint positions.
void trapezoidal::get_desired_joint_pos(KDL::JntArray& desired_joint_pos, double& current_trajec_time) const 
{
   for(int i=0; i < switch_time.size(); ++i)
   {
      if(current_trajec_time < switch_time[i])
	 for(int i=0; i< desired_joint_pos.rows(); ++i)
	    desired_joint_pos(i) = qi_(i) + 0.5 * (current_trajec_time * current_trajec_time * max_acc[i] * KDL::sign(qf_qi(i)));
	 
      else if(current_trajec_time < (tf[i] - switch_time[i]))
	 for(int i=0; i< desired_joint_pos.rows(); ++i)
	    desired_joint_pos(i) = qi_(i) + (current_trajec_time - (0.5 * switch_time[i])) * max_vel[i] * KDL::sign(qf_qi(i));
	        
      else if(current_trajec_time < tf[i])
	 for(int i=0; i< desired_joint_pos.rows(); ++i)
	    desired_joint_pos(i) = qf_(i) - 0.5 * (tf[i] - current_trajec_time) * (tf[i] - current_trajec_time) * max_acc[i] * KDL::sign(qf_qi(i));
		 
      else return;
   }
}

//Current desired joint velocities.
void trapezoidal::get_desired_joint_vel(KDL::JntArray& desired_joint_vel, double& current_trajec_time) const
{
   for(int i=0; i < switch_time.size(); ++i)
   {
      if(current_trajec_time < switch_time[i])
	 for(int i=0; i< desired_joint_vel.rows(); ++i)
	    desired_joint_vel(i) = current_trajec_time * max_acc[i] * KDL::sign(qf_qi(i));
	 
      else if(current_trajec_time < (tf[i] - switch_time[i]))
	  for(int i=0; i< desired_joint_vel.rows(); ++i)
	     desired_joint_vel(i) = 1;
	       
      else if(current_trajec_time < tf[i])
	  for(int i=0; i< desired_joint_vel.rows(); ++i)
	     desired_joint_vel(i) = (tf[i] - current_trajec_time) * max_acc[i] * KDL::sign(qf_qi(i));
	  
      else desired_joint_vel(i) = 0;       
   }
}

//Current desired joint accelerations.
void trapezoidal::get_desired_joint_acc(KDL::JntArray& desired_joint_acc, double& current_trajec_time) const 
{
   for(int i=0; i < switch_time.size(); ++i)
   {
      if(current_trajec_time < switch_time[i])
	 for(int i=0; i< desired_joint_acc.rows(); ++i)
	    desired_joint_acc(i) = KDL::sign(qf_qi(i)) * max_acc[i];
	 
      else if(current_trajec_time < (tf[i] - switch_time[i]))
	 for(int i=0; i< desired_joint_acc.rows(); ++i)
	    desired_joint_acc(i) = 0;
	       
      else if(current_trajec_time < tf[i])
	 for(int i=0; i< desired_joint_acc.rows(); ++i)
            desired_joint_acc(i) = max_acc[i] * KDL::sign(qf_qi(i));
	       
      else desired_joint_acc(i) = 0;
   }
}


bool getJointSpaceVelProfile(std::string vel_prof_name, std::unique_ptr<joint_space_vel_prof::velocityProfile>& vel_prof, double m_vel[])
{
   if(vel_prof_name == "trapezoidal"){
      // vel_prof.reset(new joint_space_vel_prof::trapezoidal(7));
      // RTT::log(RTT::Info) << "Creating trapezoidal joint space velocity profile." << RTT::endlog();
      return true;
   }

   else if(vel_prof_name == "quintic_polynomial"){
      vel_prof.reset(new joint_space_vel_prof::quinticPolynomial(7, m_vel));
      // RTT::log(RTT::Info) << "Creating quintic_polynomial joint space velocity profile." << RTT::endlog();
      std::cout << "Build trajectory profile..." << std::endl;
      return true;
   }
   else if(vel_prof_name == "cubic_polynomial"){
      // vel_prof.reset(new joint_space_vel_prof::cubicPolynomial(7, m_vel));
      // RTT::log(RTT::Info) << "Creating cubic_polynomial joint space velocity profile." << RTT::endlog();
      return true;
   }

   return false;
}

}

