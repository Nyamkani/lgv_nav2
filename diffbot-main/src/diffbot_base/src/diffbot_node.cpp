#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"


#include "diffbot_msg/srv/encoderservice.hpp"
#include "std_srvs/srv/empty.hpp"



//for std::bind
using std::placeholders::_1;
using std::placeholders::_2;

class encoder : public rclcpp::Node
{
  public:
    encoder(): Node("diffbot_base_encoder")
    {
        //encoder subscription from diffbot_hw_board via MicroXRCE agent, via serial communication
        encoder_subs = this->create_subscription<std_msgs::msg::Int32MultiArray>("diffbot_hw_encoder", 1, std::bind(&encoder::get_encoder_accumulated_value, this, _1));

        //send encoder value to ros2_control_hardware_interface using ros2 service node
        send_interface = this->create_service<diffbot_msg::srv::Encoderservice>("diffbot_base_encoder", std::bind(&encoder::send_to_encoder_service_interface,         
        this,_1,_2));
    }


    void send_to_encoder_service_interface(const std::shared_ptr<diffbot_msg::srv::Encoderservice::Request>  request
       ,std::shared_ptr<diffbot_msg::srv::Encoderservice::Response>  response)
    {
      if(request->state)
      {
        //send to ros2_control_hardware
        response->to_encoder_left = encoder_left_accumulated;
        response->to_encoder_right = encoder_right_accumulated;                
        response->to_encoder_accumulate_count = sampling_duration_accumulated;                

        //clear acummulated value
        encoder_left_accumulated = 0;
        encoder_right_accumulated = 0;
        sampling_duration_accumulated = 0 ;
      }
    }

  private:
    int32_t encoder_left_accumulated;
    int32_t encoder_right_accumulated;
    int32_t sampling_duration_accumulated;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_subs;      
    rclcpp::Service<diffbot_msg::srv::Encoderservice>::SharedPtr send_interface;


    void get_encoder_accumulated_value(const std_msgs::msg::Int32MultiArray & diffbot_hw_encoder)
    {
      //The value and sampling_duration_accumulated are accumulated until updated.
      encoder_left_accumulated = encoder_left_accumulated + diffbot_hw_encoder.data[0];
      encoder_right_accumulated = encoder_right_accumulated + diffbot_hw_encoder.data[1];
      sampling_duration_accumulated = sampling_duration_accumulated + diffbot_hw_encoder.data[2];

      //RCLCPP_INFO(this->get_logger(), "msg.data: '%.5f'   leftw travell acc: '%.5f'  accumm cnt '%.1f'", msg.data, encoder_left_buffer, sampling_duration);        
    }
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto diffbot_base = std::make_shared<encoder>();
  rclcpp::spin(diffbot_base);
  rclcpp::shutdown();
  return 0;
}