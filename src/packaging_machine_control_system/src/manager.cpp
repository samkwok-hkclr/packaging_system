#include "packaging_machine_control_system/manager.hpp"

PackagingMachineManager::PackagingMachineManager(
  std::weak_ptr<rclcpp::Executor> executor,
  const std::string & node_name, 
  const std::string & node_namespace,
  const rclcpp::NodeOptions & options) 
: Node(std::move(node_name), node_namespace, options),
  executor_(executor)
{
  status_sub_ = create_subscription<PackagingMachineStatus>(
    "machine_status", 
    10, 
    std::bind(&PackagingMachineManager::status_cb, 
      this, 
      std::placeholders::_1));
  
  packagin_result_sub_ = create_subscription<PackagingResult>(
    "packaging_result", 
    10, 
    std::bind(&PackagingMachineManager::packaging_result_cb, 
      this, 
      std::placeholders::_1));

  action_client_manager_ = std::make_shared<rclcpp_components::ComponentManager>(
    executor_, 
    action_client_manager_node_name,
    rclcpp::NodeOptions().use_global_arguments(false));

  executor_->add_node(action_client_manager_);

  service_ = this->create_service<PackagingOrderSrv>(
    packaging_order_service_name, 
    std::bind(&PackagingMachineManager::packaging_order_handle, 
      this, 
      std::placeholders::_1, 
      std::placeholders::_2));

  load_node_client_ = this->create_client<LoadNode>(load_node_service_name);
  unload_node_client_ = this->create_client<UnloadNode>(unload_node_service_name);
  list_node_client_ = this->create_client<ListNodes>(list_nodes_service_name);

  while (!load_node_client_->wait_for_service(std::chrono::seconds(1))) 
  {
    RCLCPP_ERROR(this->get_logger(), "Load Node Service not available!");
  }
  while (!unload_node_client_->wait_for_service(std::chrono::seconds(1))) 
  {
    RCLCPP_ERROR(this->get_logger(), "Unload Node Service not available!");
  }
  while (!list_node_client_->wait_for_service(std::chrono::seconds(1))) 
  {
    RCLCPP_ERROR(this->get_logger(), "List Nodes Service not available!");
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Packaging Machine Manager is up.");
}

void PackagingMachineManager::status_cb(const PackagingMachineStatus::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex_);
  packaging_machine_status[msg->packaging_machine_id] = *msg;
}

void PackagingMachineManager::packaging_result_cb(const PackagingResult::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex_);
  auto target = std::find_if(curr_client_.begin(), curr_client_.end(),
    [msg](const std::pair<uint32_t, uint64_t>& entry) {
      return entry.first == msg->order_id;
    });
  
  if (target != curr_client_.end()) 
  {
    RCLCPP_INFO(this->get_logger(), "The target action clinet is found in manager");

    auto list_nodes_srv_request = std::make_shared<ListNodes::Request>();

    using ListNodesServiceResponseFuture = rclcpp::Client<ListNodes>::SharedFuture;

    auto list_nodes_response_received_cb = [this, target](ListNodesServiceResponseFuture future) {
      auto srv_result = future.get();
      if (srv_result) 
      {
        auto target_unique_id = std::find(srv_result->unique_ids.begin(), srv_result->unique_ids.end(), target->second);
        if (target_unique_id != srv_result->unique_ids.end()) 
        {
          RCLCPP_INFO(this->get_logger(), "The target unique_id is found. Try to unload it now.");

          auto unload_node_srv_request = std::make_shared<UnloadNode::Request>();
          unload_node_srv_request->unique_id = target->second;

          using UnloadNodeServiceResponseFuture = rclcpp::Client<UnloadNode>::SharedFuture;

          auto response_received_cb = [this, target](UnloadNodeServiceResponseFuture future) {
            auto srv_result = future.get();
            if (srv_result) 
            {
              RCLCPP_INFO(this->get_logger(), "The action client is removed (unique_id: %ld)", target->second);
            } else {
              RCLCPP_ERROR(this->get_logger(), "Service call failed or returned no result");
            }
          };

          auto future = unload_node_client_->async_send_request(unload_node_srv_request, response_received_cb);
        } else {
          RCLCPP_ERROR(this->get_logger(), "The action client is not found in ListNodes Service");
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Service call failed or returned no result");
      }
    };

    auto list_nodes_future = list_node_client_->async_send_request(list_nodes_srv_request, list_nodes_response_received_cb);
  } else {
    RCLCPP_ERROR(this->get_logger(), "The target action clinet is not found in manager.");
  }
}

void PackagingMachineManager::packaging_order_handle(
  const std::shared_ptr<PackagingOrderSrv::Request> request, 
  std::shared_ptr<PackagingOrderSrv::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "service handle");

  // TODO: simpfy this function
  // find the idle packaging machine
  uint8_t target_machine_id = 0;
  for (const auto& entry : packaging_machine_status)
  {
    if (entry.second.packaging_machine_state == PackagingMachineStatus::IDLE)
    {
      target_machine_id = entry.second.packaging_machine_id;
      break;
    }
  }

  if (target_machine_id == 0) 
  {
    response->success = false;
    response->message = "Packaging Machines are busy";
    RCLCPP_ERROR(this->get_logger(), "Packaging Machines are busy");
  }

  auto load_node_srv_request = std::make_shared<LoadNode::Request>();
  load_node_srv_request->package_name = "packaging_machine_control_system";
  load_node_srv_request->plugin_name = "action_client::PackagingMachineActionClient";
  load_node_srv_request->node_name = "action_client_" + std::to_string(request->order_id);

  rcl_interfaces::msg::Parameter p1;
  rcl_interfaces::msg::ParameterValue p1_v;
  p1_v.type = 2;
  p1_v.integer_value = target_machine_id;
  p1.name = "packaging_machine_id";
  p1.value = p1_v;

  rcl_interfaces::msg::Parameter p2;
  rcl_interfaces::msg::ParameterValue p2_v;
  p2_v.type = 2;
  p2_v.integer_value = request->order_id;
  p2.name = "order_id";
  p2.value = p2_v;

  rcl_interfaces::msg::Parameter p3;
  rcl_interfaces::msg::ParameterValue p3_v;
  p3_v.type = 2;
  p3_v.integer_value = request->material_box_id;
  p3.name = "material_box_id";
  p3.value = p3_v;

  rcl_interfaces::msg::Parameter p4;
  rcl_interfaces::msg::ParameterValue p4_v;
  p4_v.type = 9;
  p4_v.string_array_value.assign(request->print_info.begin(), request->print_info.end());
  p4.name = "print_info";
  p4.value = p4_v;

  load_node_srv_request->parameters.push_back(p1);
  load_node_srv_request->parameters.push_back(p2);
  load_node_srv_request->parameters.push_back(p3);
  load_node_srv_request->parameters.push_back(p4);

  using ServiceResponseFuture = rclcpp::Client<LoadNode>::SharedFuture;

  auto response_received_callback = [this, request, response](ServiceResponseFuture future) {
    auto srv_result = future.get();
    if (srv_result) {
      std::pair<uint32_t, uint64_t> _pair(request->order_id, srv_result->unique_id);
      {
        const std::lock_guard<std::mutex> lock(this->mutex_);
        curr_client_.push_back(_pair);
      }
      RCLCPP_INFO(this->get_logger(), "Added a ID: %d ", srv_result->success);
      response->success = srv_result->success;

    } else {
      response->success = srv_result->success;
      response->message = "Service call failed or returned no result";
      RCLCPP_ERROR(this->get_logger(), "Service call failed or returned no result");
    }
  };

  auto future = load_node_client_->async_send_request(load_node_srv_request, response_received_callback);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<PackagingMachineManager>(
    exec, 
    "packaging_machine_manager",
    "",
    options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();
  rclcpp::shutdown();
}

