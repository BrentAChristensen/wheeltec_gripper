#pragma once

// ROS
#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
//#include "wheeltec_gripper_driver/ArduinoDriver.h"
#include <sstream>

const unsigned int NUM_JOINTS = 1;

using namespace std;
namespace wheeltec_hardware_interface
{
/// \brief Hardware interface for a robot
class WheeltecHardwareInterface  : public hardware_interface::RobotHW
{
  public:



    WheeltecHardwareInterface(ros::NodeHandle& nh);

   void init()
   {
      ROS_INFO("connecting to serial arduino:");
  	  std::string serial_port;
	    int baudrate;
	    nh_.getParam("/wheeltec/hardware_driver/serial_port", serial_port);
	    nh_.getParam("/wheeltec/hardware_driver/baudrate", baudrate);
	   // driver_.init(serial_port, baudrate);
   }
    void write(ros::Duration elapsed_time)
    {
       if (prev_cmd[0]!=cmd[0])
       {
          prev_cmd[0]=cmd[0];
          //driver_.update(cmd[0]);
       } 
    }  
    void read(ros::Duration elapsed_time) 
    {
      pos[0] = cmd[0];
      vel[0] =10.0;
    }

    ros::Time get_time() 
    {
      prev_update_time = curr_update_time;
      curr_update_time = ros::Time::now();
      return curr_update_time;
    }

    ros::Duration get_period() 
    {
      return curr_update_time - prev_update_time;
    }

  private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface position_joint_interface_;
    double cmd[NUM_JOINTS];
    double pos[NUM_JOINTS];
    double vel[NUM_JOINTS];
    double eff[NUM_JOINTS];
    double prev_cmd[NUM_JOINTS];

    bool running_;
    double diff_gripper_width;
  
    ros::Time curr_update_time, prev_update_time;
    ros::NodeHandle nh_;
   // wheeltec_gripper_driver::ArduinoDriver driver_;
	
};  // class

WheeltecHardwareInterface::WheeltecHardwareInterface(ros::NodeHandle &nh) : nh_(nh)
  {
    
    // Intialize raw data
    std::fill_n(pos, NUM_JOINTS, 0.0);
    std::fill_n(vel, NUM_JOINTS, 1.0);
    std::fill_n(eff, NUM_JOINTS, 30.0);
    std::fill_n(cmd, NUM_JOINTS, 0.0);
    std::fill_n(prev_cmd, NUM_JOINTS,0.0);

      ROS_INFO("starting hardware joint state controller:");
      hardware_interface::JointStateHandle state_handle("left_finger_joint", &pos[0], &vel[0], &eff[0]);
      jnt_state_interface.registerHandle(state_handle);


      ROS_INFO("starting hardware position controller:");
      hardware_interface::JointHandle jointPositionHandle(state_handle, &cmd[0]);
      position_joint_interface_.registerHandle(jointPositionHandle);


      ROS_INFO("Registring both Interface with the hardware manager");
      registerInterface(&jnt_state_interface);
      registerInterface(&position_joint_interface_);



 }
}


