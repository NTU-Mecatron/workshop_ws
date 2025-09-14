#include "workshop_main/minimal_server.hpp"

MinimalServer::MinimalServer()
	: Node("minimal_server")
{
	service_ = this->create_service<workshop_interfaces::srv::AddThreeInts>(
		"add_three_ints",
		std::bind(&MinimalServer::handle_add_three_ints, this, _1, _2)
	);
}

void MinimalServer::handle_add_three_ints(
	const std::shared_ptr<workshop_interfaces::srv::AddThreeInts::Request> request,
	std::shared_ptr<workshop_interfaces::srv::AddThreeInts::Response> response
)
{
	response->sum = request->a + request->b + request->c;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %d" " b: %d" " c: %d",
				request->a, request->b, request->c);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", response->sum);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalServer>());
  rclcpp::shutdown();
  return 0;
}