
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class BaseNode : public rclcpp::Node
{
  public:
    BaseNode()
    :Node("test")
    {
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      cmd_vel_sub=this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&BaseNode::cmd_vel_callback, this, std::placeholders::_1));
      
    }
    ~BaseNode()
    {

    }
  private:    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
    {
      RCLCPP_INFO(this->get_logger(), "cmd_vel msg received");
      (void)vel;
    }
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseNode>());
  rclcpp::shutdown();
  return 0;
}