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

#pragma once

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointStiffness.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_msgs/JointDamping.h>
#include <std_msgs//Time.h>
#include <std_msgs//Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <vector>

#include <string>
#include <mutex>

namespace robot_interface {
  
  extern ros::Time last_update_time;

  template <typename ROSMSG>
  class robotHandler {
  public:
    robotHandler() : is_new(false) {}
    
    void set_value(const ROSMSG& value) 
    {
      mutex.lock();
      data = value;
      is_new = true;
      mutex.unlock();
    }
    
    bool get_value(ROSMSG& value) 
    {
      bool was_new = false;
      mutex.lock();
      value = data;
      was_new = is_new;
      is_new = false;
      mutex.unlock();
      
      return was_new;
    }
    
    bool has_new_value() 
    {
      return is_new;
    }
    
    ROSMSG get_value_unsynchronized() 
    {
      return data;
    }

    
  private:
    ROSMSG data;
    bool is_new;
    std::mutex mutex;
  };
  

  template <typename ROSMSG>
  class robotStateHandler : public robotHandler<ROSMSG> {
  public:
    void init(const std::string& topic, ros::NodeHandle &nh) {
      
      // ros::NodeHandle nh;
      ROS_INFO_STREAM(topic);
      subscriber = nh.subscribe<ROSMSG>(topic, 1, &robotStateHandler<ROSMSG>::set, this);
      ROS_INFO_STREAM("Subscribed to topic...");

    }
    
    bool has_new_value() {
      return robotHandler<ROSMSG>::has_new_value();
    }
    
    void set(ROSMSG value) { 
      last_update_time = ros::Time::now();
      robotHandler<ROSMSG>::set_value(value);
    }
    
    bool get(ROSMSG& value) {
      return robotHandler<ROSMSG>::get_value(value);        
    }
  private:
    ros::Subscriber subscriber;

  };

  template <typename ROSMSG>
  class robotCommandHandler : public robotHandler<ROSMSG> {

  public:
    void init(const std::string& topic, ros::NodeHandle &nh) {
      // ros::NodeHandle nh;
      ROS_INFO_STREAM(topic);
      publisher = nh.advertise<ROSMSG>(topic, 1);
      ROS_INFO_STREAM("init Publisher...");
    }
    
    void set(const ROSMSG& value) {
      robotHandler<ROSMSG>::set_value(value);
    }
    
    ROSMSG get() {
      return robotHandler<ROSMSG>::get_value_unsynchronized();
    }
    
    void publishIfNew() {
      static ROSMSG msg;
      // && robotHandler<ROSMSG>::get_value(msg)
      robotHandler<ROSMSG>::get_value(msg);
      if (publisher.getNumSubscribers())
      {
        publisher.publish(msg);
      }
    }

  private:
    ros::Publisher publisher;
 
  };
}

  /*template <typename ROSMSG>
  class robotStateHandler {
  public:
    void init(const std::string& topic, ros::NodeHandle &nh) {
      // ros::NodeHandle nh;
      subscriber = nh.subscribe<ROSMSG>(topic, 1, &iiwaStateHolder<ROSMSG>::set, this);
    }
    
    bool has_new_value() {
      return holder.has_new_value();
    }
    
    void set(ROSMSG value) {
      last_update_time = ros::Time::now();
      holder.set_value(value);
    }
    
    bool get(ROSMSG& value) {
      return holder.get_value(value);        
    }
  private:
    robotHandler<ROSMSG> handler;
    ros::Subscriber subscriber;
  };
  
  
  template <typename ROSMSG>
  class robotCommandHandler {
  public:
    void init(const std::string& topic, ros::NodeHandle &nh) {
      // ros::NodeHandle nh;
      publisher = nh.advertise<ROSMSG>(topic, 1);
      // std::cout << "Init Publishing..." <<std::endl;
    }
    
    void set(const ROSMSG& value) {
      handler.set_value(value);
    }
    
    ROSMSG get() {
      return handler.get_value_unsynchronized();
    }
    
    void publishIfNew() {
      static ROSMSG msg;
      if (publisher.getNumSubscribers() && holder.get_value(msg))
        // std::cout << "Publishing..." <<std::endl;
        publisher.publish(msg);
    }
  private:
    ros::Publisher publisher;
    robotHandler<ROSMSG> handler;
  };*/
   
