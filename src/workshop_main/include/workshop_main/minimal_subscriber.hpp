#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber();

private:
    void topic_callback(const std_msgs::msg::String & msg) const;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};