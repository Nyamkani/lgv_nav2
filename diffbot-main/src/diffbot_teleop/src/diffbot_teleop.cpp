#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>



#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71


#define LINEAR_VELOCITY_CMD_VAL 0.2
#define ANGULAR_VELOCITY_CMD_VAL 0.05




class diffbot_teleop
{
public:
  diffbot_teleop(std::shared_ptr<rclcpp::Node> nh);
  void keyLoop();

private:

  
  std::shared_ptr<rclcpp::Node> nh_;
  double linear_, angular_, l_scale_, a_scale_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  
};

diffbot_teleop::diffbot_teleop(std::shared_ptr<rclcpp::Node> nh):
  nh_(nh),
  linear_(0),
  angular_(0),
  l_scale_(1),
  a_scale_(1)
{
//   nh_.param("scale_angular", a_scale_, a_scale_);
//   nh_.param("scale_linear", l_scale_, l_scale_);

  //twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped",1);
  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
//   ros::shutdown();
  rclcpp::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
//   ros::init(argc, argv, "teleop_turtle");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("diffbot_teleop");
  diffbot_teleop teleop_turtle(node);

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void diffbot_teleop::keyLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the diffbot.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(::read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    // ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        std::cout << "LEFT" << std::endl;
        angular_ = ANGULAR_VELOCITY_CMD_VAL;
        dirty = true;
        break;
      case KEYCODE_R:
        // ROS_DEBUG("RIGHT");
        std::cout << "RIGHT" << std::endl;
        angular_ = -ANGULAR_VELOCITY_CMD_VAL;
        dirty = true;
        break;
      case KEYCODE_U:
        // ROS_DEBUG("UP");
        std::cout << "UP" << std::endl;
        linear_ = LINEAR_VELOCITY_CMD_VAL;
        dirty = true;
        break;
      case KEYCODE_D:
        // ROS_DEBUG("DOWN");
        std::cout << "DOWN" << std::endl;
        linear_ = -LINEAR_VELOCITY_CMD_VAL;
        dirty = true;
        break;
    }
   

    geometry_msgs::msg::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_->publish(twist);    
      dirty=false;
    }
  }


  return;
}







































/*
namespace diffbot_teleop
{
  class diffbot_teleopkey
  {
    public:
      exlpicit diffbot_teleopkey(std::shared_ptr<rclcpp::Node> nh);

      void KeyLoop();


    private:
      std::shared_ptr<rclcpp::Node> nh_;
      double linear_;
      double angular_;
      double l_scale_;
      double a_scale_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  }


  diffbot_teleopkey::diffbot_teleopkey(std::shared_ptr<rclcpp::Node> nh):
  nh_(nh),
  linear_(),
  angular_(),
  l_scale_(),
  a_scale_()
  {
    twist_pub_=nh_->create_publisher<geometry_msgs::msg::Twist>("/diff_drive_controller/cmd_vel_unstamped",1);
  }


  int kfd = 0;
  struct termios cooked, raw;

  void quit(int sig)
  {
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    rclcpp::shutdown();
    exit(0);
  }





void diffbot_teleop::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(::read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    // ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        std::cout << "LEFT" << std::endl;
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        // ROS_DEBUG("RIGHT");
        std::cout << "RIGHT" << std::endl;
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        // ROS_DEBUG("UP");
        std::cout << "UP" << std::endl;
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        // ROS_DEBUG("DOWN");
        std::cout << "DOWN" << std::endl;
        linear_ = -1.0;
        dirty = true;
        break;
    }
   

    geometry_msgs::msg::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_->publish(twist);    
      dirty=false;
    }
  }


  return;
}


}//namespace of diff_bot

*/