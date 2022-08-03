#include "turn_on_robot/base_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "serial/serial.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <thread>
#include <future>

#define FRAME_HEADER      0X7B       // Frame header
#define FRAME_TAIL        0X7D       // Frame tail
#define SEND_DATA_SIZE    9          // The length of data sent by ROS to the lower machine 
#define RECEIVE_DATA_SIZE    9          // The length of data sent by the lower machine to ROS

class BaseNode : public rclcpp::Node
{
  public:
    BaseNode()
    :Node("base_node")
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos, std::bind(&BaseNode::cmd_vel_callback, this, std::placeholders::_1));
      odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      // set up  serial connection
      try{
        arduino_serial.setPort("/dev/ttyACM0");
        arduino_serial.setBaudrate(115200);
        serial::Timeout _time = serial::Timeout::simpleTimeout(5000);
        arduino_serial.setTimeout(_time);
        arduino_serial.open();
      }
      
      catch(serial::IOException& e){
        RCLCPP_ERROR(this->get_logger(), e.what());
        RCLCPP_ERROR(this->get_logger(), "An error occurred while establishing serial connection with Arduino uno.");
      }
      
      if(arduino_serial.isOpen()){
        RCLCPP_INFO(this->get_logger(), "arduino serial port opened");
        future_ = exit_signal_.get_future();
        poll_thread_ = std::thread(&BaseNode::pollThread, this);
      }

      
    }
    ~BaseNode()
    {
      exit_signal_.set_value();
      poll_thread_.join();
      arduino_serial.close();
    }
  private:
    void pollThread()
    {
      std::future_status status;
      int recv_count = 0;
      bool start_frame = false;
      uint8_t recv_data[RECEIVE_DATA_SIZE]={0}; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
      uint8_t t=0;
      float px=0;
      float py=0;
      float pz=0;
      std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
      do {
        if(arduino_serial.available()){
          arduino_serial.read(&t, 1);
          if(start_frame){
            recv_data[recv_count++]=t;
          }
          else if(t==FRAME_HEADER)
          {
            start_frame = true;
            recv_data[recv_count++]=t;
          }
          if(recv_count==RECEIVE_DATA_SIZE){
            recv_count = 0;
            start_frame = false;
            //RCLCPP_INFO(this->get_logger(), "end frame");
            if(t==FRAME_TAIL){
              // finish receiving a frame
              if(recv_data[7]==(recv_data[0]^recv_data[1]^recv_data[2]^recv_data[3]^recv_data[4]^recv_data[5]^recv_data[6])){
                // received frame data correct
                //RCLCPP_INFO(this->get_logger(), "received data from arduino");
                float x = (int16_t)((recv_data[1]<<8)|recv_data[2])/1000.0;
                float y = (int16_t)((recv_data[3]<<8)|recv_data[4])/1000.0;
                float z = (int16_t)((recv_data[5]<<8)|recv_data[6])/1000.0;
                float sampling_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()-last_time).count()/1000000.0;
                //RCLCPP_INFO(this->get_logger(),"%f",sampling_time);
                px+=((x * cos(pz) - y * sin(pz)) * sampling_time); //Calculate the displacement in the X direction, unit: m 
                py+=((x * sin(pz) + y * cos(pz)) * sampling_time); //Calculate the displacement in the Y direction, unit: m 
                pz+=(z * sampling_time); //The angular displacement about the Z axis, in rad 
                //RCLCPP_INFO(this->get_logger(),"%lf %lf %lf",px,py,pz);

                tf2::Quaternion odom_quat;
                odom_quat.setRPY(0,0,pz);

                geometry_msgs::msg::TransformStamped tfStamped;
                tfStamped.header.stamp = this->now();
                tfStamped.header.frame_id = "odom";
                tfStamped.child_frame_id = "base_link";
                tfStamped.transform.translation.x = px;
                tfStamped.transform.translation.y = py;
                tfStamped.transform.translation.z = 0.0;
                tfStamped.transform.rotation = tf2::toMsg(odom_quat);
                tf_broadcaster_->sendTransform(tfStamped);


                nav_msgs::msg::Odometry odom;
                odom.header.stamp = this->now();
                odom.header.frame_id = "odom";
                odom.pose.pose.position.x = px;
                odom.pose.pose.position.y = py;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = tf2::toMsg(odom_quat);
                odom.child_frame_id = "base_link";
                odom.twist.twist.linear.x = x;
                odom.twist.twist.linear.y = y;
                odom.twist.twist.angular.z = z;

                if(x== 0&&y== 0&&z== 0){
                  //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
                  //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
                  memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
                  memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
                }
                else{
                  //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
                  //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
                  memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
                  memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));       
                }
                odom_publisher->publish(odom);
                //RCLCPP_INFO(this->get_logger(), "odom publised");
                last_time = std::chrono::steady_clock::now();
              }else{
                //RCLCPP_INFO(this->get_logger(), "data error");
              }
            }
          }
        }
        status = future_.wait_for(std::chrono::seconds(0));
      } while (status == std::future_status::timeout);
    }
    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
    {
      //RCLCPP_INFO(this->get_logger(), "cmd_vel msg received");
      int16_t transition;
  
      uint8_t send_data[SEND_DATA_SIZE];
      send_data[0] = FRAME_HEADER;

      transition = vel->linear.x*1000;
      send_data[2] = transition;
      send_data[1] = transition>>8;
      
      transition = vel->linear.y*1000;
      send_data[4] = transition;
      send_data[3] = transition>>8;
      
      transition = vel->angular.z*1000;
      send_data[6] = transition;
      send_data[5] = transition>>8;

      send_data[7] = send_data[0]^send_data[1]^send_data[2]^send_data[3]^send_data[4]^send_data[5]^send_data[6];
      send_data[8] = FRAME_TAIL;

      try
      {
        arduino_serial.write(send_data, SEND_DATA_SIZE); // Send data to arduino uno via serial port 
      }
      catch (serial::IOException& e)
      {
        RCLCPP_ERROR(this->get_logger(), e.what());
        RCLCPP_ERROR(this->get_logger(),"Unable to send data through serial port"); // If sending data fails, an error message is printed
      }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    serial::Serial arduino_serial;
    std::thread poll_thread_;
    std::shared_future<void> future_;
    std::promise<void> exit_signal_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseNode>());
  rclcpp::shutdown();
  return 0;
}