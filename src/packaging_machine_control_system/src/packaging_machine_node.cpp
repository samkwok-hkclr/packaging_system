#include "packaging_machine_control_system/packaging_machine_node.hpp"

PackagingMachineNode::PackagingMachineNode(const rclcpp::NodeOptions& options)
: Node("packaging_machine_node", options)
{
  status_ = std::make_shared<PackagingMachineStatus>();
  info_ = std::make_shared<PackagingMachineInfo>();

  this->declare_parameter<uint8_t>("packaging_machine_id", 0);
  this->declare_parameter<bool>("simulation", false);
  this->get_parameter("packaging_machine_id", status_->packaging_machine_id);
  this->get_parameter("simulation", sim_);

  status_->header.frame_id = "Packaging Machine";
  status_->packaging_machine_state = PackagingMachineStatus::IDLE;

  info_->rs_state.reserve(NO_OF_REED_SWITCHS);
  info_->valve_state.reserve(NO_OF_VALVES);
  info_->ph_state.reserve(NO_OF_PHOTOELECTRIC_SENSERS);

  status_timer_ = this->create_wall_timer(1s, std::bind(&PackagingMachineNode::pub_status_cb, this));

  // add a "/" prefix to topic name avoid adding a namespace
  status_publisher_ = this->create_publisher<PackagingMachineStatus>("/machine_status", 10); 

  motor_status_publisher_ = this->create_publisher<MotorStatus>("motor_status", 10); 

  tpdo_pub_ = this->create_publisher<COData>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/tpdo", 
    10);
  rpdo_sub_ = this->create_subscription<COData>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/rpdo", 
    10, 
    std::bind(&PackagingMachineNode::rpdo_cb, 
      this, 
      std::placeholders::_1));
  
  co_read_client_ = this->create_client<CORead>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/CO_Read");
  co_write_client_ = this->create_client<COWrite>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/CO_Write");

  conveyor_service_ = this->create_service<SetBool>(
    "conveyor_movement", 
    std::bind(&PackagingMachineNode::conveyor_handle, 
      this, 
      std::placeholders::_1, 
      std::placeholders::_2));

  using namespace std::placeholders;
  this->action_server_ = rclcpp_action::create_server<PackagingOrder>(
    this,
    "packaging_order",
    std::bind(&PackagingMachineNode::handle_goal, this, _1, _2),
    std::bind(&PackagingMachineNode::handle_cancel, this, _1),
    std::bind(&PackagingMachineNode::handle_accepted, this, _1));

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Packaging Machine Node %d is up.", status_->packaging_machine_id);

  if (sim_ == false)
  {
    while (!co_read_client_->wait_for_service(std::chrono::seconds(1))) 
    {
      RCLCPP_ERROR(this->get_logger(), "CO_Read Service not available!");
    }
    while (!co_write_client_->wait_for_service(std::chrono::seconds(1))) 
    {
      RCLCPP_ERROR(this->get_logger(), "CO_Write Service not available!");
    }

    // ctrl_stopper(0);
    // status_->conveyor_state = PackagingMachineStatus::AVAILABLE;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The CO Service client is up.");
  }
}

void PackagingMachineNode::pub_status_cb(void)
{
  const std::lock_guard<std::mutex> lock(this->mutex_);
  status_->header.stamp = this->get_clock()->now();
  status_publisher_->publish(*status_);
}


bool PackagingMachineNode::call_co_write(uint16_t _index, uint8_t _subindex, uint32_t _data)
{
  auto request = std::make_shared<COWrite::Request>();

  request->index = _index;
  request->subindex = _subindex;
  request->data = _data;

  auto future = co_write_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "OK");
    return response->success;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service CO_Write");
    return false;
  }
}

bool PackagingMachineNode::call_co_read(uint16_t _index, uint8_t _subindex, std::shared_ptr<uint32_t> _data)
{
  auto request = std::make_shared<CORead::Request>();

  request->index = _index;
  request->subindex = _subindex;

  auto future = co_read_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    *_data = response->data;
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "OK");
    return response->success;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service CO_Write");
    return false;
  }
}

void PackagingMachineNode::rpdo_cb(const COData::SharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(this->mutex_);
  switch (msg->index)
  {
  case 0x6001:
    info_->temperature = static_cast<uint8_t>(msg->data);
    break;
  case 0x6008:
    info_->temperature_ctrl = static_cast<uint16_t>(msg->data);
    break;
  case 0x6018:
    motor_status_->pkg_dis_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6026:
    motor_status_->pill_gate_loc = static_cast<uint8_t>(msg->data);
    break;
  case 0x6028:
    motor_status_->pill_gate_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6038:
    motor_status_->roller_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6046:
    motor_status_->pkg_len_loc = static_cast<uint8_t>(msg->data);
    break;
  case 0x6048:
    motor_status_->pkg_len_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6058: {
    uint8_t input = static_cast<uint8_t>(msg->data);
    for (int i = NO_OF_VALVES - 1; i >= 0; i--) 
    {
      info_->valve_state[i] = (input & 1);
      input >>= 1;
    }
    break;
  }
  case 0x6068: {
    uint8_t input = static_cast<uint8_t>(msg->data);
    for (int i = NO_OF_REED_SWITCHS - 1; i >= 0; i--) 
    {
      info_->rs_state[i] = (input & 1);
      input >>= 1;
    }
    break;
  }
  case 0x6076:
    motor_status_->squ_loc = static_cast<uint8_t>(msg->data);
    break;
  case 0x6078:
    motor_status_->squ_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6088:
    motor_status_->con_state = static_cast<uint8_t>(msg->data);
    break;
  case 0x6090: {
    uint8_t input = static_cast<uint16_t>(msg->data);
    for (int i = NO_OF_PHOTOELECTRIC_SENSERS - 1; i >= 0; i--) 
    {
      info_->ph_state[i] = (input & 1);
      input >>= 1;
    }
    break;
  }
  }
}

void PackagingMachineNode::conveyor_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  if (ctrl_conveyor(1, 0, 0, request->data))
    response->success = true;
  else
  {
    response->success = false;
    response->message = "Error to control the conveyor";
  }
}

bool PackagingMachineNode::ctrl_heater(const bool on)
{
  return call_co_write(0x6003, 0x0, on ? 1 : 0);
}

bool PackagingMachineNode::ctrl_material_box_gate(const bool open)
{
  return call_co_write(0x6050, 0x0, open ? 1 : 0); // FIXME: confirm the open code
}

bool PackagingMachineNode::ctrl_stopper(const bool protrude)
{
  return call_co_write(0x6051, 0x0, protrude ? 0 : 1); // FIXME: confirm the protrude code
}

bool PackagingMachineNode::ctrl_cutter(const bool cut)
{
  return call_co_write(0x6052, 0x0, cut ? 1 : 0); // FIXME: confirm the protrude code
}

bool PackagingMachineNode::ctrl_conveyor(
  const bool dir, 
  const uint16_t speed, 
  const bool stop_by_ph, 
  const bool ctrl)
{
  bool result = true;
  if (speed > 0)
  {
    result &= call_co_write(0x6080, 0x0, speed);
  }
  result &= call_co_write(0x6082, 0x0, dir ? 1 : 0); // FIXME: confirm the direction code
  result &= call_co_write(0x6081, 0x0, stop_by_ph ? 1 : 0);
  result &= call_co_write(0x6089, 0x0, ctrl ? 1 : 0);
  
  return result;
}

rclcpp_action::GoalResponse PackagingMachineNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PackagingOrder::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "print_info size: %lu", goal->print_info.size());

  // Assume the print_info size MUST be 28
  {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    status_->packaging_machine_state = PackagingMachineStatus::BLOCKING;
    status_->conveyor_state = PackagingMachineStatus::UNAVAILABLE;
    ctrl_stopper(1);
    if (!ctrl_conveyor(1, 0, 1, 1))
    {
      status_->packaging_machine_state = PackagingMachineStatus::ERROR;
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  // TODO: read the photoelectic sensor state to make sure the material box is stopped
  uint8_t retry = 0;
  uint8_t MAX_RETRY = 3;
  rclcpp::Rate loop_rate(1s); 
  for (; retry < MAX_RETRY && rclcpp::ok(); ++retry) 
  {
    if (info_->ph_state[0]) 
    {
      const std::lock_guard<std::mutex> lock(this->mutex_);
      status_->packaging_machine_state = PackagingMachineStatus::BUSY;
      break;
    }
    loop_rate.sleep();
  }
  if (retry >= MAX_RETRY)
  {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    status_->packaging_machine_state = PackagingMachineStatus::ERROR;
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Received goal request with order %u", goal->order_id);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PackagingMachineNode::handle_cancel(
  const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PackagingMachineNode::handle_accepted(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle)
{
  std::thread{std::bind(&PackagingMachineNode::order_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void PackagingMachineNode::order_execute(const std::shared_ptr<GaolHandlerPackagingOrder> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<PackagingOrder::Feedback>();
  auto& curr_order_status = feedback->curr_order_status;
  auto& are_drugs_fallen = feedback->are_drugs_fallen;
  auto result = std::make_shared<PackagingOrder::Result>();

  // TODO: packaging sequence 1

  // fall down the drugs

  are_drugs_fallen = false;
  goal_handle->publish_feedback(feedback);

  // TODO: packaging sequence 2
  
  // packaging the drugs internally

  // a fake handle is below
  uint8_t i = 0;
  rclcpp::Rate loop_rate(1);
  while(rclcpp::ok()) {
    if (i++ > 3)
      break;
    RCLCPP_INFO(this->get_logger(), "Non-stop looping...");
    loop_rate.sleep();
  }

  // for (uint64_t i = 1; (i < goal->print_info.size()) && rclcpp::ok(); ++i) {
  //   // Check if there is a cancel request
  //   if (goal_handle->is_canceling()) {
  //     result->order_result = curr_order_status;
  //     goal_handle->canceled(result);
  //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
  //     return;
  //   }
  //   // Update curr_order_status
  //   curr_order_status[i] = true;

  //   // Publish feedback
  //   goal_handle->publish_feedback(feedback);
  //   RCLCPP_INFO(this->get_logger(), "Publish feedback");
  //   loop_rate.sleep();
  // }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->order_result = curr_order_status;
    goal_handle->succeed(result);
    {
      const std::lock_guard<std::mutex> lock(this->mutex_);
      status_->packaging_machine_state = PackagingMachineStatus::IDLE;
    }
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<PackagingMachineNode>(options));
  rclcpp::shutdown();
}