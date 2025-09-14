#include "workshop_main/minimal_client.hpp"

MinimalClient::MinimalClient(long a, long b, long c)
    : Node("minimal_client")
{
    client_ = this->create_client<workshop_interfaces::srv::AddThreeInts>("add_three_ints");
    
    send_request(a, b, c);
}

bool MinimalClient::wait_for_server()
{
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    return true;
}

void MinimalClient::send_request(long a, long b, long c)
{
    if (!wait_for_server())
        return;

    auto request = std::make_shared<workshop_interfaces::srv::AddThreeInts::Request>();
    request->a = a;
    request->b = b;
    request->c = c;
    
    // This is an asynchronous call, it returns a future
    auto result = client_->async_send_request(request);

    // Wait for the result and process it
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Sum: %d", result.get()->sum);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service add_three_ints");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 4) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: minimal_client X Y Z");
        return 1;
    }

    auto node = std::make_shared<MinimalClient>(atol(argv[1]), atol(argv[2]), atol(argv[3]));
    // The spin_until_future_complete call in the constructor handles all the spinning needed.
    // The main function will exit after the constructor and its call to send_request complete.

    rclcpp::shutdown();
    return 0;
}