// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DIFFBOT_SYSTEM_HPP_
#define DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"



using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace diffbot_hardware
{

// define cmd publisher class
 //the node definition for the publisher to talk to micro-ROS agent

class DiffbotHWMotorLeftCmdPub : public rclcpp::Node 
{
  public:
    DiffbotHWMotorLeftCmdPub();

    void MotorCmdLeftPublish(std_msgs::msg::Int32 motor_left_pps);

  private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_left_rpm_publisher_;
};

class DiffbotHWMotorRightCmdPub : public rclcpp::Node 
{
  public:
    DiffbotHWMotorRightCmdPub();

    void MotorCmdRightPublish(std_msgs::msg::Int32 motor_right_pps);

  private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr motor_right_rpm_publisher_;
};



class DiffBotSystemHardware
  : public hardware_interface::SystemInterface
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  DIFFBOT_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  DIFFBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFFBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFFBOT_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  DIFFBOT_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  DIFFBOT_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  DIFFBOT_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;


  //std::shared_ptr<HardwareCommandPub> hw_cmd_motor_left_pps_pub_;    //make the publisher node a member
  //std::shared_ptr<HardwareCommandPub> hw_cmd_motor_right_rpm_pub_;    //make the publisher node a member

  std::shared_ptr<DiffbotHWMotorLeftCmdPub> diffbot_hw_motor_left_cmd_pub_;    //make the publisher node a member
  std::shared_ptr<DiffbotHWMotorRightCmdPub> diffbot_hw_motor_right_cmd_pub_;    //make the publisher node a member  


private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the diffbot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Store the wheeled robot position
  double base_x_, base_y_, base_theta_;

  //odometry calulate variable
  //int32_t DiffEncoderPulseLeft, DiffEncoderPulseRight;
  int32_t diff_encoder_pulse_left_, diff_encoder_pulse_right_;
  int32_t sampling_duration_;

  double delta_angle_left_, delta_angle_right_;
  double angular_position_left_, angular_position_right_;
  double angular_velocity_left_, angular_velocity_right_;

  double travelled_distance_left_, travelled_distance_right_, travelled_distance_;
  
  std_msgs::msg::Int32 diffbot_hw_motor_left_pps_ ;
  std_msgs::msg::Int32 diffbot_hw_motor_right_pps_ ;

  //protected:
    //double linearToAngular(const double &distance) const;
    //double angularToLinear(const double &angle) const;
};

}  // namespace diffbot_hardware

#endif  // DIFFBOT_HARDWARE__DIFFBOT_SYSTEM_HPP_
