
#ifndef PACKAGING_MACHINE_ACTION_CLIENT_HPP_
#define PACKAGING_MACHINE_ACTION_CLIENT_HPP_

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "smdps_msgs/msg/packaging_status.hpp"
#include "smdps_msgs/msg/packaging_result.hpp"
#include "smdps_msgs/action/packaging_order.hpp"

#define CELLS 28

using namespace std::chrono_literals;

namespace action_client
{

class PackagingMachineActionClient : public rclcpp::Node
{
  using PackagingStatus = smdps_msgs::msg::PackagingStatus;
  using PackagingResult = smdps_msgs::msg::PackagingResult;
  using PackagingOrder = smdps_msgs::action::PackagingOrder;
  using GaolHandlerPackagingOrder = rclcpp_action::ClientGoalHandle<PackagingOrder>; 

public:
  explicit PackagingMachineActionClient(const rclcpp::NodeOptions & options);
  
  inline bool is_goal_done(void) const;
  void send_goal(void);

  
private:
  std::mutex mutex_;

  uint8_t packaging_machine_id_;
  uint32_t order_id_;
  uint32_t material_box_id_;
  std::vector<std::string> print_info_;

  std::shared_ptr<PackagingStatus> packaging_status_;

  bool goal_done_;

  rclcpp::TimerBase::SharedPtr packaging_status_timer_;
  rclcpp::TimerBase::SharedPtr send_goal_timer_;
  rclcpp_action::Client<PackagingOrder>::SharedPtr client_ptr_;

  rclcpp::Publisher<PackagingResult>::SharedPtr result_pub_;
  rclcpp::Publisher<PackagingStatus>::SharedPtr packaging_status_pub_;

  void pub_status_cb(void);

  void goal_response_callback(const GaolHandlerPackagingOrder::SharedPtr & goal_handle);
  void feedback_callback(
    GaolHandlerPackagingOrder::SharedPtr,
    const std::shared_ptr<const PackagingOrder::Feedback> feedback);
  void result_callback(const GaolHandlerPackagingOrder::WrappedResult & result);
};

} // namespace action_client

#endif  // PACKAGING_MACHINE_ACTION_CLIENT_HPP_