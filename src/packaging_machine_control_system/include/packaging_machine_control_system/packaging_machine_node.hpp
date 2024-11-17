#ifndef PACKAGING_MACHINE_NODE_HPP_
#define PACKAGING_MACHINE_NODE_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "smdps_msgs/action/packaging_order.hpp"
#include "smdps_msgs/msg/packaging_machine_status.hpp"
#include "smdps_msgs/msg/packaging_machine_info.hpp"
#include "smdps_msgs/msg/motor_status.hpp"

#include "canopen_interfaces/msg/co_data.hpp"
#include "canopen_interfaces/srv/co_read.hpp"
#include "canopen_interfaces/srv/co_write.hpp"

#define GRIDS 28

#define NO_OF_REED_SWITCHS 8
#define NO_OF_VALVES 4
#define NO_OF_PHOTOELECTRIC_SENSERS 8

using namespace std::chrono_literals;

class PackagingMachineNode : public rclcpp::Node
{
public:
  using SetBool = std_srvs::srv::SetBool;

  using PackagingMachineStatus = smdps_msgs::msg::PackagingMachineStatus;
  using PackagingMachineInfo = smdps_msgs::msg::PackagingMachineInfo;
  using MotorStatus = smdps_msgs::msg::MotorStatus;
  using PackagingOrder = smdps_msgs::action::PackagingOrder;

  using GaolHandlerPackagingOrder = rclcpp_action::ServerGoalHandle<PackagingOrder>;

  using COData = canopen_interfaces::msg::COData;
  using CORead = canopen_interfaces::srv::CORead;
  using COWrite = canopen_interfaces::srv::COWrite;

  explicit PackagingMachineNode(const rclcpp::NodeOptions& options);

  void pub_status_cb(void);

  bool call_co_write(uint16_t _index, uint8_t _subindex, uint32_t _data);
  bool call_co_read(uint16_t _index, uint8_t _subindex, std::shared_ptr<uint32_t> _data);

  void conveyor_handle(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);
  
  // packaging machine operation
  bool ctrl_heater(const bool on); 
  bool ctrl_material_box_gate(const bool open); 
  bool ctrl_stopper(const bool protrude); 
  bool ctrl_cutter(const bool cut); 
  bool ctrl_conveyor(const bool dir, const uint16_t speed, const bool stop_by_ph, const bool ctrl);

  void rpdo_cb(const COData::SharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const PackagingOrder::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle);
  void handle_accepted(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle);

  void order_execute(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle);

private:
  std::mutex mutex_;

  bool sim_;
  std::shared_ptr<PackagingMachineStatus> status_;
  std::shared_ptr<MotorStatus> motor_status_;
  std::shared_ptr<PackagingMachineInfo> info_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Publisher<PackagingMachineStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<MotorStatus>::SharedPtr motor_status_publisher_;

  rclcpp::Publisher<COData>::SharedPtr tpdo_pub_;
  rclcpp::Subscription<COData>::SharedPtr rpdo_sub_;

  rclcpp::Service<SetBool>::SharedPtr conveyor_service_;

  rclcpp::Client<CORead>::SharedPtr co_read_client_;
  rclcpp::Client<COWrite>::SharedPtr co_write_client_;

  rclcpp_action::Server<PackagingOrder>::SharedPtr action_server_;
}; // class PackagingMachineNode

// RCLCPP_COMPONENTS_REGISTER_NODE(PackagingMachineNode)

#endif  // PACKAGING_MACHINE_NODE_HPP_