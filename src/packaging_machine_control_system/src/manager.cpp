#include "packaging_machine_control_system/manager.hpp"

PackagingMachineManager::PackagingMachineManager(
  std::weak_ptr<rclcpp::Executor> executor,
  const std::string & node_name, 
  const std::string & node_namespace,
  const rclcpp::NodeOptions & options) 
: Node(std::move(node_name), node_namespace, options),
  executor_(executor)
{
  status_sub_ = this->create_subscription<PackagingMachineStatus>(
    "machine_status", 
    10, 
    std::bind(&PackagingMachineManager::status_cb, 
      this, 
      std::placeholders::_1));
  
  packaging_result_sub_ = this->create_subscription<PackagingResult>(
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

  srv_cli_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  service_ = this->create_service<PackagingOrderSrv>(
    packaging_order_service_name, 
    std::bind(&PackagingMachineManager::packaging_order_handle, 
      this, 
      std::placeholders::_1, 
      std::placeholders::_2));

  unbind_order_id_publisher_ = this->create_publisher<UnbindOrderId>("unbind_order_id", 10); 

  load_node_client_ = this->create_client<LoadNode>(
    load_node_service_name,
    rmw_qos_profile_services_default,
    srv_cli_cbg_);
  unload_node_client_ = this->create_client<UnloadNode>(
    unload_node_service_name,
    rmw_qos_profile_services_default,
    srv_cli_cbg_);
  list_node_client_ = this->create_client<ListNodes>(
    list_nodes_service_name,
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

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
  if (!msg->success)
  {
    RCLCPP_ERROR(this->get_logger(), "A packaging order return error.");
    // TODO: how to handle
    return;
  }

  auto target = std::find_if(curr_client_.begin(), curr_client_.end(),
    [msg](const std::pair<uint32_t, uint64_t>& entry) {
      return entry.first == msg->order_id;
  });

  UnbindOrderId unbind_msg;
  unbind_msg.packaging_machine_id = msg->packaging_machine_id;
  unbind_msg.order_id = msg->order_id;
  unbind_msg.material_box_id = msg->material_box_id;
  unbind_order_id_publisher_->publish(unbind_msg);
  
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
          RCLCPP_INFO(this->get_logger(), "The target unique_id is found in current loaded. Try to unload it now.");

          auto unload_node_srv_request = std::make_shared<UnloadNode::Request>();
          unload_node_srv_request->unique_id = target->second;

          using UnloadNodeServiceResponseFuture = rclcpp::Client<UnloadNode>::SharedFuture;

          auto response_received_cb = [this, target](UnloadNodeServiceResponseFuture future) {
            auto srv_result = future.get();
            if (srv_result) 
            {
              curr_client_.erase(target);
              RCLCPP_INFO(this->get_logger(), "The action client is unloaded (unique_id: %ld)", target->second);
            } else 
            {
              RCLCPP_ERROR(this->get_logger(), "Service call failed or returned no result");
            }
          };
          auto future = unload_node_client_->async_send_request(unload_node_srv_request, response_received_cb);
        } else 
        {
          RCLCPP_ERROR(this->get_logger(), "The action client is not found in ListNodes Service");
        }
      } else 
      {
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
  const std::lock_guard<std::mutex> lock(this->mutex_);

  auto it = std::find_if(packaging_machine_status.begin(), packaging_machine_status.end(),
    [](const auto& entry) {
        return entry.second.packaging_machine_state == PackagingMachineStatus::IDLE;
  });

  if (it == packaging_machine_status.end()) 
  {
    response->success = false;
    response->message = "Packaging Machines are busy";
    RCLCPP_ERROR(this->get_logger(), "Packaging Machines are busy");
    return;
  }

  uint8_t target_machine_id = it->second.packaging_machine_id;

  auto load_node_srv_request = std::make_shared<LoadNode::Request>();
  load_node_srv_request->package_name = "packaging_machine_control_system";
  load_node_srv_request->plugin_name = "action_client::PackagingMachineActionClient";
  load_node_srv_request->node_name = "action_client_" + std::to_string(request->order_id);

  rcl_interfaces::msg::ParameterValue p1_v;
  p1_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  p1_v.integer_value = target_machine_id;
  rcl_interfaces::msg::Parameter p1;
  p1.name = "packaging_machine_id";
  p1.value = p1_v;

  rcl_interfaces::msg::ParameterValue p2_v;
  p2_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  p2_v.integer_value = request->order_id;
  rcl_interfaces::msg::Parameter p2;
  p2.name = "order_id";
  p2.value = p2_v;

  rcl_interfaces::msg::ParameterValue p3_v;
  p3_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  p3_v.integer_value = request->material_box_id;
  rcl_interfaces::msg::Parameter p3;
  p3.name = "material_box_id";
  p3.value = p3_v;

  rcl_interfaces::msg::ParameterValue p_cn_name_v;
  p_cn_name_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_cn_name_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    p_cn_name_v.string_array_value[i] = request->print_info[i].cn_name;
  }
  rcl_interfaces::msg::Parameter p_cn_name;
  p_cn_name.name = "cn_name";
  p_cn_name.value = p_cn_name_v;

  rcl_interfaces::msg::ParameterValue p_en_name_v;
  p_en_name_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_en_name_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    p_en_name_v.string_array_value[i] = request->print_info[i].en_name;
  }
  rcl_interfaces::msg::Parameter p_en_name;
  p_en_name.name = "en_name";
  p_en_name.value = p_en_name_v;

  rcl_interfaces::msg::ParameterValue p_date_v;
  p_date_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_date_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    p_date_v.string_array_value[i] = request->print_info[i].date;
  }
  rcl_interfaces::msg::Parameter p_date;
  p_date.name = "date";
  p_date.value = p_date_v;

  rcl_interfaces::msg::ParameterValue p_time_v;
  p_time_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_time_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    p_time_v.string_array_value[i] = request->print_info[i].time;
  }
  rcl_interfaces::msg::Parameter p_time;
  p_time.name = "time";
  p_time.value = p_time_v;

  rcl_interfaces::msg::ParameterValue p_drugs_v;
  p_drugs_v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  p_drugs_v.string_array_value.resize(CELLS);
  for (size_t i = 0; i < CELLS; i++)
  {
    auto join = [&](const std::vector<std::string>& vec, char delimiter) {
      std::ostringstream oss;
      for (size_t i = 0; i < vec.size(); ++i) 
      {
        oss << vec[i];
        if (i != vec.size() - 1) 
          oss << delimiter; // Add # delimiter except for the last element
      }
      return oss.str();
    };
    p_drugs_v.string_array_value[i] = join(request->print_info[i].drugs, '#');
  }
  rcl_interfaces::msg::Parameter p_drugs;
  p_drugs.name = "drugs";
  p_drugs.value = p_drugs_v;

  load_node_srv_request->parameters.push_back(p1);
  load_node_srv_request->parameters.push_back(p2);
  load_node_srv_request->parameters.push_back(p3);
  load_node_srv_request->parameters.push_back(p_cn_name);
  load_node_srv_request->parameters.push_back(p_en_name);
  load_node_srv_request->parameters.push_back(p_date);
  load_node_srv_request->parameters.push_back(p_time);
  load_node_srv_request->parameters.push_back(p_drugs);

  using ServiceResponseFuture = rclcpp::Client<LoadNode>::SharedFuture;

  auto response_received_cb = [this, request, response](ServiceResponseFuture future) {
    auto srv_result = future.get();
    if (srv_result) 
    {
      std::pair<uint32_t, uint64_t> _pair(request->order_id, srv_result->unique_id);
      {
        const std::lock_guard<std::mutex> lock(this->mutex_);
        curr_client_.push_back(_pair);
      }
      RCLCPP_INFO(this->get_logger(), "Loaded a action client. unique_id: %ld ", srv_result->unique_id);
    } else 
    {
      std::string err_msg = "Service call failed or returned no result";
      response->message = err_msg;
      RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    }
  };

  auto future = load_node_client_->async_send_request(load_node_srv_request, response_received_cb);

  std::future_status status = future.wait_for(500ms);
  switch (status)
  {
  case std::future_status::ready:
    response->success = true;
    break; 
  default: {
    response->success = false;
    std::string err_msg = "The Loadnode Service is wait too long.";
    response->message = err_msg;
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    break;
  }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
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

