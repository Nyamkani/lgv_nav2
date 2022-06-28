/*
 * Authors: kss (2022)
 * 
 * Based on the sicklms.cpp from the sicktoolbox_wrapper ROS package
 * and the sample code from the sicktoolbox manual.
 * 
 * Released under Apache 2.0 license.
 */ 

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"



//declare standard c++14 headers
#include <iostream>
#include <sstream>
#include <deque>
#include <thread>

//declare own ros2 headers
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <nav_msgs/msg/odometry.hpp>

//declare own package headers
#include <SickNAV350.hh>

#define DEG2RAD(x) ((x)*M_PI/180.)

const std::string SCAN_TOPIC = "/nav/scan/unfiltered";
const std::string ODOM_TOPIC = "/nav/odom/unfiltered";
const std::string Node_Name = "NAV350_Hardware_Node";

//brought from nav350 source
//for exit by executing ctr+c
bool need_exit = false;

namespace Nyamkani
{
  namespace OperatingModes
  {
    enum OperatingMode
    {
      POWERDOWN = 0,
      STANDBY = 1,
      MAPPING = 2,
      LANDMARK = 3,
      NAVIGATION = 4,
    };
  }

  enum ErrCode
  {
    Good = 0x0000,
    Warning = 0x0001, 
    Err_01 = 0x0002, //Configuration error
    Err_02 = 0x0004,
    Err_03 = 0x0008,
    Err_04 = 0x0010,
    Err_05 = 0x0020,
    Err_06 = 0x0040,
    Err_07 = 0x0080,
    Err_08 = 0x0100,
    Err_09 = 0x0200,
    Err_10 = 0x0400,
    Err_11 = 0x0800,
  };

  enum State
  {
    Stop = 0,
    Init, 
    Idle,
    Scan,
    Pub, 
    Err,
  };

  typedef OperatingModes::OperatingMode OperatingMode;

  class sick_nav350: public rclcpp::Node
  {
    
    public:
      sick_nav350(std::string node_name) : Node(node_name)
      { 
          scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(SCAN_TOPIC, 10);
          odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 50); //rclcpp::QoS(rclcpp::KeepLast(10))

          // Call on_timer function every second
          using namespace std::chrono_literals;
          //scan_pub_callback = this->create_wall_timer(100ms, std::bind(&sick_nav350::Publish_Scan_Data, this));
          //odom_pub_callback = this->create_wall_timer(25ms, std::bind(&sick_nav350::Publish_Odometry_Data, this));

          odom_broadcasters =
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);    
      }
      ~sick_nav350(){}

    private:
      int op_mode, port, wait, mask;

      std::string ipaddress;
      std::string frame_id;
      std::string scan;
      bool scan_inverted, odom_inverted, do_mapping;
      bool publish_odom;
      int sick_motor_speed = 8;//10; // Hz
      std::string target_frame_id; // the frame to be publish relative to frame_id
      std::string mobile_base_frame_id = "";
      std::string reflector_frame_id, reflector_child_frame_id;

      rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

      std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcasters;

      rclcpp::TimerBase::SharedPtr scan_pub_callback{nullptr};
      rclcpp::TimerBase::SharedPtr odom_pub_callback{nullptr};

      //pointer
      SickToolbox::SickNav350 *nav350_instance;

      tf2::Stamped<tf2::Transform>::Transform mobile_base_current_tf;
      tf2::Stamped<tf2::Transform>::Transform mobile_base_prev_tf;

      /* Define buffers for return values */
      double range_values[SickToolbox::SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};
      int intensity_values[SickToolbox::SickNav350::SICK_MAX_NUM_MEASUREMENTS] = {0};

      /* Define buffers to hold sector specific data */
      unsigned int num_measurements = {0};
      unsigned int sector_start_timestamp = {0};
      unsigned int sector_stop_timestamp = {0};
      double sector_step_angle = {0};
      double sector_start_angle = {0};
      double sector_stop_angle = {0};
      double last_time_stamp = 0;

      /*Transforms*/
      tf2::Transform odom_to_sick_tf;

      //Scans
      unsigned int last_sector_stop_timestamp;

      //states
      State state = Stop;
      //error filter
      int ErrCnt;

    private:
      void init_param()
      {
          //standard config
          this->declare_parameter("mode");
          this->declare_parameter("port");
          this->declare_parameter("ipaddress");
          this->declare_parameter("scan_inverted");
          this->declare_parameter("odom_inverted");
          this->declare_parameter("publish_odom");
          this->declare_parameter("perform_mapping");
          this->declare_parameter("scan");

          //frame id enable
          this->declare_parameter("frame_id");
          this->declare_parameter("reflector_frame_id");
          this->declare_parameter("reflector_child_frame_id");
          this->declare_parameter("target_frame_id");

          //command
          this->declare_parameter("wait_command");
          this->declare_parameter("mask_command");

          //etc
          this->declare_parameter("timer_smoothing_factor");
          this->declare_parameter("timer_error_threshold");
          this->declare_parameter("resolution");
          this->declare_parameter("scan_rate");

          //add values in params
          this->get_parameter_or<int>("mode", op_mode, 4);
          this->get_parameter_or<int>("port", port, DEFAULT_SICK_TCP_PORT);
          this->get_parameter_or<std::string>("ipaddress", ipaddress, (std::string)DEFAULT_SICK_IP_ADDRESS);
          this->get_parameter_or<bool>("scan_inverted", scan_inverted, false);
          this->get_parameter_or<bool>("odom_inverted", odom_inverted, false);
          this->get_parameter_or<bool>("publish_odom",publish_odom, false);
          this->get_parameter_or<bool>("perform_mapping", do_mapping, true);
          this->get_parameter_or<std::string>("scan", scan, "scan");
          this->get_parameter_or<std::string>("frame_id", frame_id, "map");
          this->get_parameter_or<std::string>("target_frame_id",target_frame_id, "base_link");
          this->get_parameter_or<std::string>("reflector_frame_id", reflector_frame_id, "nav350");
          this->get_parameter_or<std::string>("reflector_child_frame_id", reflector_child_frame_id, "reflector");
          this->get_parameter_or<int>("wait_command", wait, 1);
          this->get_parameter_or<int>("mask_command", mask, 2);
          this->get_parameter_or<int>("scan_rate",sick_motor_speed,8);
      }

      // TODO: refactor these functions into a common util lib (similar to code in sicklms.cpp)
      void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub, double *range_values,
                uint32_t n_range_values, int *intensity_values,
                uint32_t n_intensity_values, rclcpp::Time start,
                double scan_time, bool scan_inverted, float angle_min,
                float angle_max, std::string frame_id); 


      void createOdometryMessage(const rclcpp::Duration& time_elapsed, const tf2::Stamped<tf2::Transform>::Transform& prev_transform,
                            const tf2::Stamped<tf2::Transform>::Transform& current_transform);

      void PublishReflectorTransform(std::vector<double> x,std::vector<double> y,double th,std::vector<tf2_ros::TransformBroadcaster> odom_broadcaster,std::string frame_id,std::string child_frame_id);

      SickToolbox::SickNav350& Get_Instance()
      {
        if (nav350_instance == nullptr) nav350_instance = new SickToolbox::SickNav350(ipaddress.c_str(),port);
        return *nav350_instance;
      }

      void RecoveryFunction();

      void ErrorHandler();

      int Setup_Device();
      
      int Uninitailize();

      int Get_Data_From_Nav350();

      int Parsing_Localliazation_Data();
  
      int Publish_Scan_Data();
 
      int Publish_Odometry_Data();

      int Publish_Landmark_Data();

      int Parsing_Datas();

      int Publish_Datas();
  
      int main_operation();

      int Process();

     public:
      int main_loop()
      {
        init_param();
        auto nav350 = sick_nav350::Get_Instance();
        
        main_operation();
        return 0;
      }
  };


  //Time out iosocket error. SickThreadException


  // TODO: refactor these functions into a common util lib (similar to code in sicklms.cpp)
  void sick_nav350::publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub, double *range_values,
            uint32_t n_range_values, int *intensity_values,
            uint32_t n_intensity_values, rclcpp::Time start,
            double scan_time, bool scan_inverted, float angle_min,
            float angle_max, std::string frame_id)   
  {
    auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    scan_msg->header.stamp = start;
    scan_msg->header.frame_id = target_frame_id;

    // assumes scan window at the bottom
    if (scan_inverted) { 
      scan_msg->angle_min = angle_max;
      scan_msg->angle_max = angle_min;
    } 
    else 
    {
      scan_msg->angle_min = angle_min;
      scan_msg->angle_max = angle_max;
    }

    scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (double)(n_range_values-1);
    scan_msg->scan_time = scan_time;
    scan_msg->time_increment = scan_msg->scan_time / (double)(n_range_values);
    scan_msg->range_min = 0.1;
    scan_msg->range_max = 250.;
    
    scan_msg->ranges.resize(n_range_values);


    for (size_t i = 0; i < n_range_values; i++) 
    {
      if ((float)range_values[i] == 0.0) scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
      else scan_msg->ranges[i] = (float)range_values[i]/1000;

    }

    scan_msg->intensities.resize(n_intensity_values+1);
    for (size_t i = 0; i < n_intensity_values; i++) 
    {
      scan_msg->intensities[i] = (float)intensity_values[i];
    }
    pub->publish(*scan_msg);
  }

  void sick_nav350::createOdometryMessage(const rclcpp::Duration& time_elapsed, const tf2::Stamped<tf2::Transform>::Transform& prev_transform,
                            const tf2::Stamped<tf2::Transform>::Transform& current_transform)
  {
    auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();

    auto now_time = this->now();

    //-------------------send odometry msg.
    //add infomations
    odom_msg->header.stamp = now_time;
    odom_msg->header.frame_id = frame_id;
    //odom_msg->child_frame_id = target_frame_id;

    double dt = time_elapsed.nanoseconds();
    double dx = (current_transform.getOrigin().getX() - prev_transform.getOrigin().getX())/dt;
    double dy = (current_transform.getOrigin().getY() - prev_transform.getOrigin().getY())/dt;
    double dr = (tf2::getYaw(current_transform.getRotation()) - tf2::getYaw(prev_transform.getRotation()))/dt;

    // getting position
    odom_msg->pose.pose.position.x = current_transform.getOrigin().getX();
    odom_msg->pose.pose.position.y = current_transform.getOrigin().getY();
    odom_msg->pose.pose.position.z = 0.0f;

    // //this is function for more accurately odoms
    // //but we will use kalman filter
    // //odom_msg->pose.covariance.assign(0.0f);

    // //may this change cause unexpected error 
    // //tf2_ros::quaternionTFToMsg(current_transform.getRotation(),odom_msg.pose.pose.orientation);
    tf2::convert(current_transform.getRotation(), odom_msg->pose.pose.orientation);

    // set velocity
    odom_msg->twist.twist.linear.x = dx;
    odom_msg->twist.twist.linear.y = dy;
    odom_msg->twist.twist.linear.z = 0.0f;
    odom_msg->twist.twist.angular.x = 0.0f;
    odom_msg->twist.twist.angular.y = 0.0f;
    odom_msg->twist.twist.angular.z = dr;
    //odom_msg.twist.covariance.assign(0.0f);

    //nav350_instance->SetSpeed(dx,dy,dr,sector_start_timestamp,0);
    odom_pub->publish(*odom_msg);

    //-----------------------send transform 
    //send transform
    geometry_msgs::msg::TransformStamped s;
    
    tf2::convert(current_transform, s.transform);
    //add infomations
    s.header.stamp = now_time;
    s.header.frame_id = frame_id;
    s.child_frame_id = target_frame_id;
  

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Sending transform from "<< frame_id <<" to "<< target_frame_id);
    odom_broadcasters->sendTransform(s);  


  }

  void sick_nav350::PublishReflectorTransform(std::vector<double> x,std::vector<double> y,double th,std::vector<tf2_ros::TransformBroadcaster> odom_broadcaster,std::string frame_id,std::string child_frame_id)
  {
    //printf("\npos %.2f %.2f %.2f\n",x,y,th);
    tf2::Transform b_transforms;
    //tf2::Stamped<tf2::Transform> b_transforms;
    tf2::Quaternion tfquat; 

    tfquat.setRPY(0, 0, th);//th should be 0? no orientation information on reflectors?

    std::ostringstream childframes;
    for (int i=0;i<(int)x.size();i++)
    {
      childframes << child_frame_id << i;
      b_transforms.setRotation(tfquat);
      b_transforms.setOrigin(tf2::Vector3(-x[i] / 1000, -y[i] / 1000, 0.0));

      //Caused by no exist tf::StampedTransform() func.
      geometry_msgs::msg::TransformStamped t;

      //add infomations
      t.header.stamp = this->now();
      t.header.frame_id = frame_id;
      t.child_frame_id = childframes.str();
      tf2::convert(b_transforms, t.transform);

      odom_broadcaster[i].sendTransform(t);
      childframes.str(std::string());
      childframes.clear();
    }
  }

  int sick_nav350::Setup_Device()
  {
    /* Initialize the device */
    nav350_instance->Initialize();
    nav350_instance->GetSickIdentity();

    /* Initialize transforms */ 
    mobile_base_current_tf.getIdentity();
    mobile_base_prev_tf.getIdentity();


    // TODO: do some calls to setup the device - e.g. scan rate. Configure mapping. Configure reflectors/landmarks
    if (do_mapping)
    {
      nav350_instance->SetOperatingMode((int)OperatingModes::MAPPING);
      nav350_instance->DoMapping();
      nav350_instance->SetOperatingMode((int)OperatingModes::STANDBY);
      RCLCPP_INFO(this->get_logger(), "Sicknav50 Mapping Completed");
    }

    try
    {
      nav350_instance->SetOperatingMode((int)OperatingModes::STANDBY);
      nav350_instance->SetScanDataFormat();
      nav350_instance->SetOperatingMode(op_mode);
    }

    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Configuration error");
      return -1;
    }
    return 0;
  }

  int sick_nav350::Get_Data_From_Nav350()
  {
    //Grab the measurements (from all sectors)
    if (op_mode==3)
    {
      //ROS_INFO_STREAM("Getting landmark data");
      nav350_instance->GetDataLandMark(1,1);
    }

    else if (op_mode==4)
    {
      //ROS_DEBUG_STREAM("Getting nav data");
      nav350_instance->GetDataNavigation(wait,mask);
    }

    else
    {
      RCLCPP_INFO_STREAM(this->get_logger()," Selected operating mode does not return data... try again");
      return -1;
    }
    
    Parsing_Datas();
    return 0;
  }

  int sick_nav350::Parsing_Localliazation_Data()
  {
    /*
    * Get nav localization data
    */
    double x1 = (double) nav350_instance->PoseData_.x;
    double y1 = (double) nav350_instance->PoseData_.y;
    double phi = nav350_instance->PoseData_.phi;
    //RCLCPP_INFO(this->get_logger(), "NAV350 pose in x y alpha: pose x = %.3f, pose y = %.3f, angle  = %.3f", x1, y1, phi1/1000.0);
    
    tf2::Quaternion odomquat;
    odomquat.setRPY(0, 0, DEG2RAD(phi/1000.0));
    //odomquat.inverse();

    if(odom_inverted) odom_to_sick_tf.setOrigin(tf2::Vector3(-x1 / 1000, -y1/ 1000, 0.0));
    else odom_to_sick_tf.setOrigin(tf2::Vector3(x1 / 1000, y1/ 1000, 0.0));

    odom_to_sick_tf.setRotation(odomquat);

    return 0;
  }

  int sick_nav350::Publish_Landmark_Data()
  {
    /*
    * Get landmark data and broadcast transforms
    */
    std::vector<tf2_ros::TransformBroadcaster> landmark_broadcasters;
    int num_reflectors = nav350_instance->PoseData_.numUsedReflectors;
    int number_reflectors = nav350_instance->ReflectorData_.num_reflector;
    std::vector<double> Rx(number_reflectors), Ry(number_reflectors);
    double phi = nav350_instance->PoseData_.phi;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "NAV350 # reflectors seen:"<<number_reflectors);
    RCLCPP_DEBUG_STREAM(this->get_logger(), "NAV350 # reflectors used:"<<num_reflectors);

    for (int r=0;r<number_reflectors; r++)
    {
        Rx[r]=(double) nav350_instance->ReflectorData_.x[r];
        Ry[r]=(double) nav350_instance->ReflectorData_.y[r];
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Reflector "<<r<<" x pos: "<<Rx[r]<<" y pos: "<<Ry[r]);
    }
    landmark_broadcasters.resize(number_reflectors, this);
    PublishReflectorTransform(Rx, Ry, DEG2RAD(phi/1000.0), landmark_broadcasters, reflector_frame_id, reflector_child_frame_id);
    return 0;
  }

  int sick_nav350::Publish_Scan_Data()
  {
    /* Set Scan Publisher configuration */
    if(sector_start_timestamp > last_time_stamp)
    {
      last_time_stamp = sector_start_timestamp;
      
      double scan_duration = 1./sick_motor_speed;
      rclcpp::Time start_scan_time = this->get_clock()->now()- rclcpp::Duration(scan_duration);

      if(odom_inverted)
      {
        sector_start_angle-=180;
        sector_stop_angle-=180;
      }
      /* Publishih Scan msg.*/
      publish_scan(scan_pub, range_values, num_measurements, intensity_values, num_measurements, start_scan_time, scan_duration, scan_inverted,
          DEG2RAD((float)sector_start_angle), DEG2RAD((float)sector_stop_angle), target_frame_id);
      last_sector_stop_timestamp = sector_stop_timestamp;
    }
    return 0;
  }

  int sick_nav350::Publish_Odometry_Data()
  {
    // publishing odometry
    if(publish_odom)
    {
      double scan_duration = 1./(double)sick_motor_speed;
      mobile_base_current_tf = odom_to_sick_tf;
      createOdometryMessage(rclcpp::Duration::from_seconds(scan_duration), mobile_base_prev_tf, mobile_base_current_tf);
      mobile_base_prev_tf = mobile_base_current_tf;
    }
    return 0;
  }

  int sick_nav350::Parsing_Datas()
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Getting sick range/scan measurements");
    nav350_instance->GetSickMeasurements(range_values, intensity_values, 
                                        &num_measurements, &sector_step_angle, 
                                        &sector_start_angle, &sector_stop_angle, 
                                        &sector_start_timestamp, &sector_stop_timestamp);
    Parsing_Localliazation_Data();
    return 0;
  }

  int sick_nav350::Publish_Datas()
  {
    int status = 0;
      // Look up for the transformation between target_frame and turtle2 frames
    //   // and send velocity commands for turtle2 to reach target_frame
    // try 
    // {
    //   //geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform("base_link", "nav350", tf2::TimePointZero);
    //   geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform("base_link", "nav350", tf2::TimePointZero);
    // } 
    // catch (tf2::TransformException & ex) 
    // {
    //   RCLCPP_INFO(this->get_logger(), "Could not transform map to odom, Maybe URDF is not detected");
    //   return 1;
    // }

    std::vector<std::thread> threads;
    threads.emplace_back(std::thread(&sick_nav350::Publish_Scan_Data, this));
    threads.emplace_back(std::thread(&sick_nav350::Publish_Odometry_Data, this));
    threads.emplace_back(std::thread(&sick_nav350::Publish_Landmark_Data, this));
    for (auto& thread : threads) thread.join();

    return status;
  }

  int sick_nav350::Process()
  {
    // std::vector<std::thread> threads;
    // threads.emplace_back(std::thread(&sick_nav350::Get_Data_From_Nav350, this));
    // threads.emplace_back(std::thread(&sick_nav350::Publish_Datas, this));
    // for (auto& thread : threads) thread.join(); 
    Get_Data_From_Nav350();
    Publish_Datas();

    return 0;
  }

  int sick_nav350::main_operation()
  {
    rclcpp::Rate loop_rate(sick_motor_speed);
    try 
    {
      Setup_Device();  
      while (rclcpp::ok() && !need_exit)
      {
        Process();
        loop_rate.sleep();
        rclcpp::spin_some(shared_from_this());
      }
    }
    catch(...)
    {
      RCLCPP_ERROR(this->get_logger(), "Error");
      return -1;
    }
    Uninitailize();
    rclcpp::spin_some(shared_from_this());
    return 0;
  }

  int sick_nav350::Uninitailize()
  {
    /* Uninitialize the device */
    try
    {
      nav350_instance->Uninitialize();
    }

    catch(...)
    {
      std::cerr << "Uninitialize failed!" << std::endl;
      return -1;
    }
    return 0;
  }
}

void ExitHandler(int sig)
{
    (void)sig;
    need_exit = true;
}

int main(int argc, char *argv[])
{ 
  rclcpp::init(argc, argv);
  auto sick_nav350_publisher = std::make_shared<Nyamkani::sick_nav350>(Node_Name);
  signal(SIGINT,ExitHandler);
  int ret = sick_nav350_publisher->main_loop();
  rclcpp::shutdown();
  return ret;
}

