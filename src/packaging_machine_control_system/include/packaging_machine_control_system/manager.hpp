#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include <map>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "composition_interfaces/srv/list_nodes.hpp"

#include "smdps_msgs/msg/packaging_machine_status.hpp"
#include "smdps_msgs/msg/packaging_result.hpp"

#include "smdps_msgs/srv/packaging_order.hpp"

class PackagingMachineManager : public rclcpp::Node
{
public:
  using PackagingMachineStatus = smdps_msgs::msg::PackagingMachineStatus;
  using PackagingResult = smdps_msgs::msg::PackagingResult;

  using PackagingOrderSrv = smdps_msgs::srv::PackagingOrder;

  using LoadNode = composition_interfaces::srv::LoadNode;
  using UnloadNode = composition_interfaces::srv::UnloadNode;
  using ListNodes = composition_interfaces::srv::ListNodes;

  explicit PackagingMachineManager(
    std::weak_ptr<rclcpp::Executor> executor,
    const std::string & node_name,
    const std::string & node_namespace,
    const rclcpp::NodeOptions & options);

  void packaging_order_handle(
    const std::shared_ptr<PackagingOrderSrv::Request> request, 
    std::shared_ptr<PackagingOrderSrv::Response> response);

private:
  std::mutex mutex_;

  // order_id, unique_id
  std::vector<std::pair<uint32_t, uint64_t>> curr_client_;

  rclcpp::Service<PackagingOrderSrv>::SharedPtr service_;
  rclcpp::Subscription<PackagingMachineStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<PackagingResult>::SharedPtr packagin_result_sub_;

  std::map<uint8_t, PackagingMachineStatus> packaging_machine_status;

  const std::string action_client_manager_node_name = "action_client_manager";
  const std::string load_node_service_name = action_client_manager_node_name + "/_container/load_node";
  const std::string unload_node_service_name = action_client_manager_node_name + "/_container/unload_node";
  const std::string list_nodes_service_name = action_client_manager_node_name + "/_container/list_nodes";

  const std::string packaging_order_service_name = "packaging_order";

  rclcpp::Client<LoadNode>::SharedPtr load_node_client_;
  rclcpp::Client<UnloadNode>::SharedPtr unload_node_client_;
  rclcpp::Client<ListNodes>::SharedPtr list_node_client_;

  void status_cb(const PackagingMachineStatus::SharedPtr msg);
  void packaging_result_cb(const PackagingResult::SharedPtr msg);

protected:
  std::shared_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<rclcpp_components::ComponentManager> action_client_manager_;

};


#endif  // MANAGER_HPP_