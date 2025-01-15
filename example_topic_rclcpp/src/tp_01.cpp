#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"  // 使用标准消息 Int32
#include "rm_interfaces/msg/gimbal_cmd.hpp"

class GimbalSubscriber : public rclcpp::Node
{
public:
    GimbalSubscriber() : Node("gimbal_subscriber")
    {
        RCLCPP_INFO(this->get_logger(), "gimbal_msg_monitor node launching...");
        rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();
        subscription_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
            "armor_solver/cmd_gimbal", qos_profile, std::bind(&GimbalSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const rm_interfaces::msg::GimbalCmd::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received: %f", msg->yaw);
    }

    rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
