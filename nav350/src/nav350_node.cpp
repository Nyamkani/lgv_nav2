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

const std::string ODOM_TOPIC = "odom";
const std::string Node_Name = "sick_nav350_node";

//brought from nav350 source
//for exit by executing ctr+c
bool need_exit = false;

namespace Nyamkani
{
  // A complimentary filter to get a (much) better time estimate, does not
  // calibrate out constant network latency delays, but does get rid of 
  // timming jitter to get better timing estimates than the 
  // communicated clock resolution (which is only 1ms)
  class smoothtime { 
      protected:
          rclcpp::Time smoothtime_prev, smoothed_timestamp;
          double time_smoothing_factor;
          double error_threshold;
      public:
      //!
      smoothtime(){
          time_smoothing_factor = 0.95; /// slowly skew the clocks into sync
          error_threshold = .50; /// 50% jitter is acceptable for , discard data otherwise.
      }
      //! Between 0 and 1, bigger is smoother
      void set_smoothing_factor(double smoothing_factor)
      {
          time_smoothing_factor = smoothing_factor;
      }
      //! Between 0 and 1, threshold on jitter acceptability, higher accepts more jitter before discarding
      void set_error_threshold(double err_threshold)
      {
          error_threshold = err_threshold;
      }
      rclcpp::Time smooth_timestamp(rclcpp::Time recv_timestamp, rclcpp::Duration expctd_dur) {
      //if (smoothtime_prev.is_zero() == true) {
      if ((smoothtime_prev.seconds() == 0 && smoothtime_prev.nanoseconds() == 0 ) == true) {
        smoothed_timestamp = recv_timestamp;
      } else {
        smoothed_timestamp = smoothtime_prev + expctd_dur;
        double err = (recv_timestamp - smoothed_timestamp).nanoseconds();            //.toSec-> .nanoseconds
        double time_error_threshold = expctd_dur.nanoseconds() * error_threshold;    //.toSec-> .nanoseconds
        if ((time_smoothing_factor > 0) && (fabs(err) < time_error_threshold)){
          rclcpp::Duration correction = rclcpp::Duration(err * (1 - time_smoothing_factor));
          smoothed_timestamp += correction;
        } else {
          // error too high, or smoothing disabled - set smoothtime to last timestamp
          smoothed_timestamp = recv_timestamp;
        }
      }
      smoothtime_prev = smoothed_timestamp;
      return smoothed_timestamp;
    }
  };

  class averager {
  protected:
    std::deque<double> deq;
    unsigned int max_len;
    double sum;
  public:
    averager(int max_len = 50){
      this->max_len = max_len;
    }
    void add_new(double data) {
      deq.push_back(data);
      sum += data;
      if (deq.size() > max_len) {
        sum -= deq.front();
        deq.pop_front();
      }
    }
    double get_mean() {
      return sum/deq.size();
    }
  };

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
    Working, 
    Err,

  };

  typedef OperatingModes::OperatingMode OperatingMode;

  class sick_nav350: public rclcpp::Node
  {
    
    public:
      sick_nav350(std::string node_name) : Node(node_name)
      { 
          scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
          odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(ODOM_TOPIC, 10);

          tf_buffer =
                std::make_unique<tf2_ros::Buffer>(this->get_clock());
          tf_listener =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
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
      double sick_step_angle = 1.5;//0.5;//0.25; // deg (0.125 = no gaps between spots)
      double active_sector_start_angle = 0;
      double active_sector_stop_angle = 360;//269.75;
      double smoothing_factor, error_threshold;
      const double TRANSFORM_TIMEOUT = 20.0f;
      const double POLLING_DURATION = 0.05f;
      std::string target_frame_id; // the frame to be publish relative to frame_id
      std::string mobile_base_frame_id = "";
      std::string reflector_frame_id, reflector_child_frame_id;

      rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

      std::unique_ptr<tf2_ros::Buffer> tf_buffer;
      std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcasters;

      std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
      SickToolbox::SickNav350 *nav350_instance;

      tf2::Stamped<tf2::Transform>::Transform mobile_base_current_tf = tf2::Stamped<tf2::Transform>::Transform::getIdentity();
      tf2::Stamped<tf2::Transform>::Transform mobile_base_prev_tf = tf2::Stamped<tf2::Transform>::Transform::getIdentity();

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
      double last_time_stamp=0;

      //
      rclcpp::Time now;

      /*Transforms*/
      tf2::Transform odom_to_sick_tf;
      tf2::Transform odom_to_target_tf;

      //Scans
      double scan_duration;
      unsigned int last_sector_stop_timestamp;
      double full_duration;
      rclcpp::Time start_scan_time;

      smoothtime smoothtimer = smoothtime();
      averager avg_fulldur = averager();
      averager avg_scandur = averager();

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

          this->get_parameter_or<double>("timer_smoothing_factor", smoothing_factor, 0.97);
          this->get_parameter_or<double>("timer_error_threshold", error_threshold, 0.5);
          this->get_parameter_or<double>("resolution", sick_step_angle, 1.0);
          this->get_parameter_or<int>("scan_rate",sick_motor_speed,8);
      }

      // TODO: refactor these functions into a common util lib (similar to code in sicklms.cpp)
      void publish_scan(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& pub, double *range_values,
                uint32_t n_range_values, int *intensity_values,
                uint32_t n_intensity_values, rclcpp::Time start,
                double scan_time, bool scan_inverted, float angle_min,
                float angle_max, std::string frame_id,
                unsigned int sector_start_timestamp); 


      void createOdometryMessage(const rclcpp::Duration& time_elapsed, const tf2::Stamped<tf2::Transform>::Transform& prev_transform,
                            const tf2::Stamped<tf2::Transform>::Transform& current_transform);

      void PublishReflectorTransform(std::vector<double> x,std::vector<double> y,double th,std::vector<tf2_ros::TransformBroadcaster> odom_broadcaster,std::string frame_id,std::string child_frame_id);

      tf2::Quaternion createQuaternionFromYaw(double yaw);

      SickToolbox::SickNav350& Get_Instance()
      {
        if (nav350_instance == nullptr) nav350_instance = new SickToolbox::SickNav350(ipaddress.c_str(),port);
        return* nav350_instance;
      }

      void RecoveryFunction();

      void ErrorHandler();

      int Setup_Device();
      
      int Uninitailize();

      int Get_Data_From_Nav350();

      int Parsing_Localliazation_Data();
  
      int Transform_frame_to_target_frame();
 
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
            float angle_max, std::string frame_id,
            unsigned int sector_start_timestamp)   
  {
    auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

    scan_msg->header.stamp = start;
    scan_msg->header.frame_id = frame_id;

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
    scan_msg->scan_time = scan_time;//0.125;//scan_time;
    scan_msg->time_increment = scan_msg->scan_time/*scan_time*/ / n_range_values;
    scan_msg->range_min = 0.1;
    scan_msg->range_max = 250.;
    scan_msg->ranges.resize(n_range_values);


    for (size_t i = 0; i < n_range_values; i++) 
    {
        scan_msg->ranges[i] = (float)range_values[i]/1000;
    }
    scan_msg->intensities.resize(n_intensity_values);
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

    //add infomations
    odom_msg->header.stamp = now;
    odom_msg->header.frame_id = frame_id;
    odom_msg->child_frame_id = target_frame_id;

    double dt = time_elapsed.nanoseconds();
    double dx = (current_transform.getOrigin().getX() - prev_transform.getOrigin().getX())/dt;
    double dy = (current_transform.getOrigin().getY() - prev_transform.getOrigin().getY())/dt;
    double dr = (tf2::getYaw(current_transform.getRotation()) - tf2::getYaw(prev_transform.getRotation()))/dt;

    // getting position
    odom_msg->pose.pose.position.x = current_transform.getOrigin().getX();
    odom_msg->pose.pose.position.y = current_transform.getOrigin().getY();
    odom_msg->pose.pose.position.z = 0.0f;

    //this is function for more accurately odoms
    //but we will use kalman filter
    //odom_msg->pose.covariance.assign(0.0f);

    //may this change cause unexpected error 
    //tf2_ros::quaternionTFToMsg(current_transform.getRotation(),odom_msg.pose.pose.orientation);
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
  }

  void sick_nav350::PublishReflectorTransform(std::vector<double> x,std::vector<double> y,double th,std::vector<tf2_ros::TransformBroadcaster> odom_broadcaster,std::string frame_id,std::string child_frame_id)
  {
    //printf("\npos %.2f %.2f %.2f\n",x,y,th);
    tf2::Transform b_transforms;
    //tf2::Stamped<tf2::Transform> b_transforms;
    tf2::Quaternion tfquat = createQuaternionFromYaw(th);//th should be 0? no orientation information on reflectors?

    std::ostringstream childframes;
    for (int i=0;i<(int)x.size();i++)
    {
      childframes << child_frame_id << i;
      b_transforms.setRotation(tfquat);
      b_transforms.setOrigin(tf2::Vector3(-x[i] / 1000, -y[i] / 1000, 0.0));

      //Caused by no exist tf::StampedTransform() func.
      geometry_msgs::msg::TransformStamped t;

      //add infomations
      t.header.stamp = now;
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
    /*Initialize utils*/
    smoothtimer.set_smoothing_factor(smoothing_factor);
    smoothtimer.set_error_threshold(error_threshold);

    /* Initialize the device */
    nav350_instance->Initialize();
    nav350_instance->GetSickIdentity();

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
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Getting sick range/scan measurements");
    nav350_instance->GetSickMeasurements(range_values, intensity_values, &num_measurements, &sector_step_angle, &sector_start_angle, &sector_stop_angle, &sector_start_timestamp, &sector_stop_timestamp);
    Parsing_Datas();

    rclcpp::Rate loop_rate(sick_motor_speed);
    loop_rate.sleep();
    rclcpp::spin_some(shared_from_this());

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
    //odomquat.inverse();
    if(odom_inverted) 
    {
      odomquat = createQuaternionFromYaw(DEG2RAD(phi/1000.0));
      odom_to_sick_tf.setOrigin(tf2::Vector3(-x1 / 1000, -y1/ 1000, 0.0));
    }
    else
    {
      odomquat = createQuaternionFromYaw(DEG2RAD(phi/1000.0));
      odom_to_sick_tf.setOrigin(tf2::Vector3(x1 / 1000, y1/ 1000, 0.0));
    }
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

  int sick_nav350::Transform_frame_to_target_frame()
  {
    // Publish /map to /odom transform
    // converting to target frame
    odom_to_target_tf = odom_to_sick_tf;

    geometry_msgs::msg::TransformStamped s;

    //add infomations
    s.header.stamp =  now;
    s.header.frame_id = frame_id;
    s.child_frame_id = target_frame_id;
    tf2::convert(odom_to_target_tf, s.transform);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Sending transform from "<< frame_id <<" to "<< target_frame_id);
    odom_broadcasters->sendTransform(s);  
    return 0;
  }

  int sick_nav350::Publish_Scan_Data()
  {
    /*
    * Set Scan Publisher config.
    */
    if(sector_start_timestamp > last_time_stamp)
    {
      last_time_stamp = sector_start_timestamp;
      rclcpp::Time end_scan_time = now;

      scan_duration = (sector_stop_timestamp - sector_start_timestamp) * 1e-3;
      avg_scandur.add_new(scan_duration);
      scan_duration = avg_scandur.get_mean();//avg_scandur.get_mean() 0.125;

      if (last_sector_stop_timestamp == 0) {
          full_duration = 1./((double)sick_motor_speed);
      } else {
          full_duration = (sector_stop_timestamp - last_sector_stop_timestamp) * 1e-3;
      }
      avg_fulldur.add_new(full_duration);
      full_duration = avg_fulldur.get_mean();

      rclcpp::Time smoothed_end_scan_time = smoothtimer.smooth_timestamp(end_scan_time, rclcpp::Duration(full_duration));
      start_scan_time = smoothed_end_scan_time - rclcpp::Duration(scan_duration);
      if(odom_inverted)
      {
        sector_start_angle-=180;
        sector_stop_angle-=180;
      }
      /* Publishih Scan msg.*/
      publish_scan(scan_pub, range_values, num_measurements, intensity_values, num_measurements, start_scan_time, scan_duration, scan_inverted,
          DEG2RAD((float)sector_start_angle), DEG2RAD((float)sector_stop_angle), target_frame_id, sector_start_timestamp);
      last_sector_stop_timestamp = sector_stop_timestamp;
    }
    return 0;
  }

  int sick_nav350::Publish_Odometry_Data()
  {
    // publishing odometry
    if(publish_odom)
    {
        mobile_base_current_tf = odom_to_target_tf;
        createOdometryMessage(rclcpp::Duration::from_seconds(0.3), mobile_base_prev_tf, mobile_base_current_tf);
        mobile_base_prev_tf = mobile_base_current_tf;
    }
    return 0;
  }

  int sick_nav350::Parsing_Datas()
  {
    Parsing_Localliazation_Data();
    return 0;
  }

  int sick_nav350::Publish_Datas()
  {
    int status = 0;

    std::vector<std::thread> threads;
    threads.emplace_back(std::thread(&sick_nav350::Publish_Scan_Data, this));
    threads.emplace_back(std::thread(&sick_nav350::Publish_Odometry_Data, this));
    threads.emplace_back(std::thread(&sick_nav350::Transform_frame_to_target_frame, this));
    threads.emplace_back(std::thread(&sick_nav350::Publish_Landmark_Data, this));
    for (auto& thread : threads) thread.join();

    rclcpp::spin_some(shared_from_this());
    return status;
  }

  int sick_nav350::Process()
  {
    now = this->get_clock()->now();
    std::vector<std::thread> threads;
    threads.emplace_back(std::thread(&sick_nav350::Get_Data_From_Nav350, this));
    threads.emplace_back(std::thread(&sick_nav350::Publish_Datas, this));
    for (auto& thread : threads) thread.join(); 
    return 0;
  }

  int sick_nav350::main_operation()
  {
    try 
    {
      Setup_Device();  
      while (rclcpp::ok() && !need_exit)
      {
        Process();
      }
      Uninitailize();
    }
    catch(...)
    {
      RCLCPP_ERROR(this->get_logger(), "Error");
      return -1;
    }
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

 //This function will check errors and Return to init function
  void sick_nav350::RecoveryFunction()
  {

  }

  //This function will check error counts and publishing errors
  void sick_nav350::ErrorHandler()
  {

  }

  //no functions in tf2
  //newly created 
  tf2::Quaternion sick_nav350::createQuaternionFromYaw(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, (yaw));
    //return tf2::toMsg(q);
    return q;
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

