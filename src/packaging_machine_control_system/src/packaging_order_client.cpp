#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "smdps_msgs/srv/packaging_order.hpp"

#define CELLS 28

// This node is used for creating packaging orders for testing purposes.

class PackagingOrderClient : public rclcpp::Node {
public:
	PackagingOrderClient() : Node("package_order_client") {
    client_ = this->create_client<smdps_msgs::srv::PackagingOrder>("packaging_order");
	}

	void send_request() {
		auto request = std::make_shared<smdps_msgs::srv::PackagingOrder::Request>();
		request->order_id = 1234; 
		request->material_box_id = 5678;
		request->requester_id = 90;

		for (size_t i = 0; i < CELLS; ++i) {
			request->print_info[i].cn_name = "CN Name " + std::to_string(i + 1);
			request->print_info[i].en_name = "EN Name " + std::to_string(i + 1);
			request->print_info[i].date = "2023-11-27";
			request->print_info[i].time = "12:00";
			request->print_info[i].drugs = {"Drug A", "Drug B"};
		}

		while (!client_->wait_for_service(std::chrono::seconds(1))) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
		}

		auto result_future = client_->async_send_request(request);
		if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
			rclcpp::FutureReturnCode::SUCCESS) {
			RCLCPP_INFO(this->get_logger(), "Response: %s", result_future.get()->message.c_str());
		} else {
			RCLCPP_ERROR(this->get_logger(), "Failed to call service");
		}
	}

private:
	rclcpp::Client<smdps_msgs::srv::PackagingOrder>::SharedPtr client_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto client_node = std::make_shared<PackagingOrderClient>();
	client_node->send_request();
	rclcpp::shutdown();
	return 0;
}