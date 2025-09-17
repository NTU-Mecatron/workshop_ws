#include "workshop_main/minimal_publisher.hpp"

MinimalPublisher::MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    timer_ = rclcpp::create_timer(
        this->get_node_base_interface(),     // Node base interface
        this->get_node_timers_interface(),   // Timers interface
        this->get_clock(),                   // Use node's clock
        rclcpp::Duration::from_seconds(0.5), // Period = 500 ms
        std::bind(&MinimalPublisher::timer_callback, this) // Callback
    );
}

void MinimalPublisher::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
