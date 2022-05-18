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



#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "diffbot_msg/srv/encoderservice.hpp"

#include "diffbot_system.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


namespace diffbot_hardware
{
  void get_encoder_values_from_serivce_node(
    int32_t& encoder_left_val,
    int32_t& encoder_right_val,
    int32_t& sampling_duration)
  {

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("diffbot_hw_encoder");
    rclcpp::Client<diffbot_msg::srv::Encoderservice>::SharedPtr client =
    node->create_client<diffbot_msg::srv::Encoderservice>("diffbot_base_encoder");

    auto request = std::make_shared<diffbot_msg::srv::Encoderservice::Request>();
    request->state =true;

    while (!client->wait_for_service(1s)) 
    {
      if (!rclcpp::ok()) 
      {
        RCLCPP_ERROR(rclcpp::get_logger("diffbot_hw_encoder"), "Interrupted while waiting for the service. Exiting."); break;
      }
      RCLCPP_INFO(rclcpp::get_logger("diffbot_hw_encoder"), "service not available, waiting again...");
    }
    auto result = client->async_send_request(request);

    // Wait for the result.
   if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {

      encoder_left_val = result.get()->to_encoder_left;
      encoder_right_val = result.get()->to_encoder_right;
      sampling_duration = result.get()->to_encoder_accumulate_count;

      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "left: %0.5f,right: %0.5f ", result.get()->to_encoder_left,result.get()->to_encoder_right);
      //RCLCPP_INFO(rclcpp::get_logger("Debug_from_hardware_node"), "lft time stamp: %5ld. val: %d, right time stamp: %5ld, val: %d ", 
      //             get_time_temp1.nanoseconds(), result.get()->to_encoder_left, get_time_temp2.nanoseconds(), result.get()->to_encoder_right);

    } 
    else 
    {
      RCLCPP_ERROR(rclcpp::get_logger("diffbot_hw_encoder"), "Failed to call service encoder");
    }
  }


DiffbotHWMotorLeftCmdPub::DiffbotHWMotorLeftCmdPub() : Node("diffbot_hw_motor_left")
{
  motor_left_rpm_publisher_ = this->create_publisher<std_msgs::msg::Int32>("diffbot_hw_motor_left_pps", 1);
}

DiffbotHWMotorRightCmdPub::DiffbotHWMotorRightCmdPub() : Node("diffbot_hw_motor_right")
{
  motor_right_rpm_publisher_ = this->create_publisher<std_msgs::msg::Int32>("diffbot_hw_motor_right_pps", 1);
}

void DiffbotHWMotorLeftCmdPub::MotorCmdLeftPublish(std_msgs::msg::Int32 motor_left_pps)
{
  motor_left_rpm_publisher_->publish(motor_left_pps);
}

void DiffbotHWMotorRightCmdPub::MotorCmdRightPublish(std_msgs::msg::Int32 motor_right_pps)
{
  motor_right_rpm_publisher_->publish(motor_right_pps);
}








CallbackReturn DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  diffbot_hw_motor_left_cmd_pub_ = std::make_shared<DiffbotHWMotorLeftCmdPub>();  
  diffbot_hw_motor_right_cmd_pub_ = std::make_shared<DiffbotHWMotorRightCmdPub>();


  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


  //init parameter
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  hw_positions_[0] = 0.0;
  hw_positions_[1] = 0.0;

  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;  

  hw_velocities_[0] = 0.0;
  hw_velocities_[1] = 0.0;
  
  diff_encoder_pulse_left_ = 0;
  diff_encoder_pulse_right_ = 0;
  sampling_duration_ = 0;

  delta_angle_left_ = 0;
  delta_angle_right_ = 0;  

  angular_position_left_ =0;
  angular_position_right_ =0;
  angular_velocity_left_ =0;
  angular_velocity_right_ =0;

  travelled_distance_left_ =0;
  travelled_distance_right_ =0;
  travelled_distance_ = 0;

  diffbot_hw_motor_left_pps_.data = 0;
  diffbot_hw_motor_right_pps_.data = 0;   


  

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffBotSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }


  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}




#define ENCODER_RESOLUTION  112572
#define RAD_TO_PPS 17916.39103
#define MILISEC_TO_SEC 0.001




hardware_interface::return_type DiffBotSystemHardware::read()
{

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Reading...");

  double radius = 0.1;  // radius of the wheels
  double dist_w = 0.57;   // distance between the wheels

  //get encoder current value
  get_encoder_values_from_serivce_node(diff_encoder_pulse_left_, diff_encoder_pulse_right_, sampling_duration_);


  RCLCPP_INFO( rclcpp::get_logger("DiffBotSystemHardware"), "Got value from thread: %.5f, %.5f .",hw_positions_[0],hw_positions_[1]);  
  //RCLCPP_INFO( rclcpp::get_logger("DiffBotSystemHardware"), "Got value from diffbot_node: %d, %d .",CurrentEncoderPulseLeft, CurrentEncoderPulseRight);
  //RCLCPP_INFO( rclcpp::get_logger("DiffBotSystemHardware"), "diff_left: %d, diff_right: %d .",diff_encoder_pulse_left_, diff_encoder_pulse_right_);

  if( sampling_duration_ != 0 ) //some encoder message exist
  {
    angular_position_left_ =  (double)((double)diff_encoder_pulse_left_/ENCODER_RESOLUTION) * 2.0 * M_PI  ; //in radian
    angular_position_right_ =  (double)((double)diff_encoder_pulse_right_/ENCODER_RESOLUTION) * 2.0 * M_PI  ;  // in radian

    angular_velocity_left_ = ((double)angular_position_left_)/( sampling_duration_ * MILISEC_TO_SEC);   // rad/s
    angular_velocity_right_ = ((double)angular_position_right_)/( sampling_duration_ * MILISEC_TO_SEC);  // rad/s
  }

  //RCLCPP_INFO( rclcpp::get_logger("diffbot_encoder_test"), "angular_position_left: %.5f, angular_position_right: %.5f .",angular_position_left, angular_position_right);

  //update joint state velocity @radian
  hw_velocities_[0] = angular_velocity_left_;
  hw_velocities_[1] = angular_velocity_right_;

  //update joint state - wheel angular pos @radian
  hw_positions_[0] = hw_positions_[0] + angular_position_left_;
  hw_positions_[1] = hw_positions_[1] + angular_position_right_;

  //calculate linear distance [meter]
  travelled_distance_left_ = angular_position_left_*radius;
  travelled_distance_right_ = angular_position_right_*radius;
  travelled_distance_ = 0.5*(travelled_distance_left_+travelled_distance_right_);

  //RCLCPP_INFO( rclcpp::get_logger("DiffBotSystemHardware"), "travelled_distance_left:%.5f, travelled_distance_right:%.5f. ",travelled_distance_left,travelled_distance_right);

  //update x,y,heading
  double base_dx = travelled_distance_ * cos(base_theta_);
  double base_dy = travelled_distance_ * sin(base_theta_);
  double base_dtheta = (travelled_distance_right_ - travelled_distance_left_) / dist_w;

  //normalize angle [radian]
  base_dtheta = fmod(base_dtheta, 2.0*M_PI);
  if (base_dtheta < 0) base_dtheta += 2.0*M_PI;

  //update diffbot x,y,theta [meter,radian]
  base_x_ += base_dx;
  base_y_ += base_dy;
  base_theta_ += base_dtheta;

  //normalize angle [radian]
  base_theta_ = fmod(base_theta_, 2.0*M_PI);
  if (base_theta_ < 0) base_theta_ += 2.0*M_PI;

  
  RCLCPP_INFO(
    rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully read! (%.5f,%.5f,%.5f)",
    base_x_, base_y_, base_theta_);


  return hardware_interface::return_type::OK;
}





hardware_interface::return_type diffbot_hardware::DiffBotSystemHardware::write()
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");


  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
  }
  
  //publish to topic
  diffbot_hw_motor_right_pps_.data = (int32_t)(hw_commands_[1]*RAD_TO_PPS);
  diffbot_hw_motor_left_pps_.data = (int32_t)(hw_commands_[0]*RAD_TO_PPS);

  //publish to diffbot_hw_motorcmd_interface
  diffbot_hw_motor_left_cmd_pub_->MotorCmdLeftPublish(diffbot_hw_motor_left_pps_);      
  diffbot_hw_motor_right_cmd_pub_->MotorCmdRightPublish(diffbot_hw_motor_right_pps_);  


  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully written!");  

  return hardware_interface::return_type::OK;
}



}  // namespace diffbot_hardware


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffbot_hardware::DiffBotSystemHardware, hardware_interface::SystemInterface)

