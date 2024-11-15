
#ifndef PACKAGING_MACHINE_ACTION_CLIENT_HPP_
#define PACKAGING_MACHINE_ACTION_CLIENT_HPP_

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "smdps_msgs/msg/packaging_result.hpp"
#include "smdps_msgs/action/packaging_order.hpp"

#define GRIDS 2

using namespace std::chrono_literals;

namespace action_client
{

class PackagingMachineActionClient : public rclcpp::Node
{
  using PackagingResult = smdps_msgs::msg::PackagingResult;
  using PackagingOrder = smdps_msgs::action::PackagingOrder;
  using GaolHandlerPackagingOrder = rclcpp_action::ClientGoalHandle<PackagingOrder>; 
  
public:
  explicit PackagingMachineActionClient(const rclcpp::NodeOptions & options);
  
  bool is_goal_done() const;
  void send_goal();

private:
  uint8_t packaging_machine_id_;
  uint32_t order_id_;
  uint32_t material_box_id_;
  std::vector<std::string> print_info_;

  bool goal_done_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<PackagingOrder>::SharedPtr client_ptr_;

  rclcpp::Publisher<PackagingResult>::SharedPtr result_pub_;

  void goal_response_callback(const GaolHandlerPackagingOrder::SharedPtr & goal_handle);
  void feedback_callback(
    GaolHandlerPackagingOrder::SharedPtr,
    const std::shared_ptr<const PackagingOrder::Feedback> feedback);
  void result_callback(const GaolHandlerPackagingOrder::WrappedResult & result);
};

} // namespace action_client

#endif  // PACKAGING_MACHINE_ACTION_CLIENT_HPP_