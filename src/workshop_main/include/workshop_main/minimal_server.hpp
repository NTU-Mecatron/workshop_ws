#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "workshop_interfaces/srv/add_three_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MinimalServer : public rclcpp::Node
{
public:
	MinimalServer();

private:
	void handle_add_three_ints(
		const std::shared_ptr<workshop_interfaces::srv::AddThreeInts::Request> request,
		std::shared_ptr<workshop_interfaces::srv::AddThreeInts::Response> response
	);
	
    rclcpp::Service<workshop_interfaces::srv::AddThreeInts>::SharedPtr service_;
};