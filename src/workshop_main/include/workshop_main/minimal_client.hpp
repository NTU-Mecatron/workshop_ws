#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "workshop_interfaces/srv/add_three_ints.hpp"

class MinimalClient : public rclcpp::Node
{
public:
    MinimalClient(long a, long b, long c);

private:
    bool wait_for_server();
    void send_request(long a, long b, long c);

    rclcpp::Client<workshop_interfaces::srv::AddThreeInts>::SharedPtr client_;
};