#include "packaging_machine_control_system/packaging_machine_node.hpp"

PackagingMachineNode::PackagingMachineNode(const rclcpp::NodeOptions& options)
: Node("packaging_machine_node", options)
{
  status_ = std::make_shared<PackagingMachineStatus>();
  info_ = std::make_shared<PackagingMachineInfo>();
  printer_config_ = std::make_shared<Config>();

  this->declare_parameter<uint8_t>("packaging_machine_id", 0);
  this->declare_parameter<bool>("simulation", false);
  this->get_parameter("packaging_machine_id", status_->packaging_machine_id);
  this->get_parameter("simulation", sim_);

  this->declare_parameter<uint16_t>("vendor_id", 0);
  this->declare_parameter<uint16_t>("product_id", 0);
  this->declare_parameter<std::string>("serial", "");
  this->declare_parameter<uint8_t>("endpoint_in", 0);
  this->declare_parameter<uint8_t>("endpoint_out", 0);
  this->declare_parameter<int>("timeout", 0);
  this->declare_parameter<uint8_t>("dots_per_mm", 0);
  this->declare_parameter<uint8_t>("direction", 0);
  this->declare_parameter<int>("total", 0);
  this->declare_parameter<int>("interval", 0);
  this->declare_parameter<bool>("offset_x", false);
  this->declare_parameter<bool>("offset_y", false);

  this->get_parameter("vendor_id", printer_config_->vendor_id);
  this->get_parameter("product_id", printer_config_->product_id);
  this->get_parameter("serial", printer_config_->serial);
  this->get_parameter("endpoint_in", printer_config_->endpoint_in);
  this->get_parameter("endpoint_out", printer_config_->endpoint_out);
  this->get_parameter("timeout", printer_config_->timeout);
  this->get_parameter("dots_per_mm", printer_config_->dots_per_mm);
  this->get_parameter("direction", printer_config_->direction);
  this->get_parameter("total", printer_config_->total);
  this->get_parameter("interval", printer_config_->interval);
  this->get_parameter("offset_x", printer_config_->offset_x);
  this->get_parameter("offset_y", printer_config_->offset_y);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Printer is up.");
  status_->header.frame_id = "Packaging Machine";
  status_->packaging_machine_state = PackagingMachineStatus::IDLE;

  info_->rs_state.reserve(NO_OF_REED_SWITCHS);

  srv_cli_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rpdo_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions rpdo_options;
  rpdo_options.callback_group = rpdo_cbg_;

  status_timer_ = this->create_wall_timer(1s, std::bind(&PackagingMachineNode::pub_status_cb, this));

  // add a "/" prefix to topic name avoid adding a namespace
  status_publisher_ = this->create_publisher<PackagingMachineStatus>("/machine_status", 10); 

  motor_status_publisher_ = this->create_publisher<MotorStatus>("motor_status", 10); 
  unbind_req_publisher_ = this->create_publisher<UnbindRequest>("unbind_request", 10); 

  tpdo_pub_ = this->create_publisher<COData>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/tpdo", 
    10);
  rpdo_sub_ = this->create_subscription<COData>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/rpdo", 
    10,
    std::bind(&PackagingMachineNode::rpdo_cb, this, std::placeholders::_1),
    rpdo_options);
  
  co_read_client_ = this->create_client<CORead>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/CO_Read",
    rmw_qos_profile_services_default,
    srv_cli_cbg_);
  co_write_client_ = this->create_client<COWrite>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/CO_Write",
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  heater_service_ = this->create_service<SetBool>(
    "heater_operation", 
    std::bind(&PackagingMachineNode::heater_handle, 
      this, 
      std::placeholders::_1, 
      std::placeholders::_2));

  stopper_service_ = this->create_service<SetBool>(
    "stopper_operation", 
    std::bind(&PackagingMachineNode::stopper_handle, 
      this, 
      std::placeholders::_1, 
      std::placeholders::_2));

  conveyor_service_ = this->create_service<SetBool>(
    "conveyor_operation", 
    std::bind(&PackagingMachineNode::conveyor_handle, 
      this, 
      std::placeholders::_1, 
      std::placeholders::_2));

  this->action_server_ = rclcpp_action::create_server<PackagingOrder>(
    this,
    "packaging_order",
    std::bind(&PackagingMachineNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&PackagingMachineNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&PackagingMachineNode::handle_accepted, this, std::placeholders::_1));

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

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The CO Service client is up.");
  }

  auto cmd = get_print_label_cmd("testing", 0, 0);
  for (int i = 0; i < 3; i++)
  {
    printer_ = std::make_shared<Printer>(
    printer_config_->vendor_id, 
    printer_config_->product_id, 
    printer_config_->serial);
    init_printer_config();
    printer_->runTask(cmd);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "runTask");
    std::this_thread::sleep_for(3s);
    printer_.reset();;
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
    info_->stopper           = (input >> 3) & 0x1;
    info_->material_box_gate = (input >> 2) & 0x1;
    info_->cutter            = (input >> 1) & 0x1;
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
    info_->conveyor        = (input >> 7) & 0x1;
    info_->squeeze         = (input >> 6) & 0x1;
    info_->squeeze_home    = (input >> 5) & 0x1;
    info_->roller_step     = (input >> 4) & 0x1;
    info_->roller_home     = (input >> 3) & 0x1;
    info_->pill_gate_home  = (input >> 2) & 0x1;
    info_->pkg_len_level_1 = (input >> 1) & 0x1;
    info_->pkg_len_level_2 = input & 0x1;
    break;
  }
  }
}

void PackagingMachineNode::init_packaging_machine()
{
  ctrl_heater(1);
  ctrl_stopper(0);
  status_->conveyor_state = PackagingMachineStatus::AVAILABLE;

  // if ()
}

void PackagingMachineNode::wait_for_idle_motor(
  const uint8_t & motor_state, 
  const uint8_t waiting_rate)
{
  rclcpp::Rate rate(waiting_rate);
  while (rclcpp::ok())
  {
    {
      const std::lock_guard<std::mutex> lock(this->mutex_);
      if (motor_state == MotorStatus::IDLE)
        break;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for motor");
    rate.sleep();
  }
}

void PackagingMachineNode::heater_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  if (ctrl_heater(request->data))
    response->success = true;
  else
  {
    response->success = false;
    response->message = "Error to control the conveyor";
  }
}

void PackagingMachineNode::stopper_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  if (ctrl_stopper(request->data))
    response->success = true;
  else
  {
    response->success = false;
    response->message = "Error to control the conveyor";
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

bool PackagingMachineNode::ctrl_stopper(const bool protrude)
{
  return call_co_write(0x6050, 0x0, protrude ? 0 : 1);
}

bool PackagingMachineNode::ctrl_material_box_gate(const bool open)
{
  return call_co_write(0x6051, 0x0, open ? 1 : 0);
}

bool PackagingMachineNode::ctrl_cutter(const bool cut)
{
  return call_co_write(0x6052, 0x0, cut ? 1 : 0);
}

bool PackagingMachineNode::ctrl_pkg_dis(
  const float length, 
  const bool feed, 
  const bool ctrl
)
{
  bool result = true;

  result &= call_co_write(0x6011, 0x0, static_cast<uint32_t>(PULSES_PER_REV * length / (2 * M_PI * PKG_DIS_RADIUS)));
  result &= call_co_write(0x6012, 0x0, feed ? 0 : 1); // Set to 0 to feed the package out
  result &= call_co_write(0x6019, 0x0, ctrl ? 1 : 0);

  return result;
}

bool PackagingMachineNode::ctrl_pill_gate(
  const float length, 
  const bool open, 
  const bool ctrl
)
{
  bool result = true;

  result &= call_co_write(0x6021, 0x0, static_cast<uint32_t>(PULSES_PER_REV * length / (2 * M_PI * PILL_GATE_RADIUS)));
  result &= call_co_write(0x6022, 0x0, open ? 1 : 0);
  result &= call_co_write(0x6029, 0x0, ctrl ? 1 : 0);

  return result;
}

bool PackagingMachineNode::ctrl_squeezer(
  const bool squeeze, 
  const bool ctrl
)
{
  bool result = true;
  
  if (!ctrl) 
  {
    result &= call_co_write(0x6079, 0x0, 0);
    return result;
  }

  if (squeeze) 
  {
    result &= call_co_write(0x6072, 0x0, 0);
    result &= call_co_write(0x6073, 0x0, 1);
  }  else {
    result &= call_co_write(0x6072, 0x0, 1);
    result &= call_co_write(0x6073, 0x0, 0);
  }

  result &= call_co_write(0x6079, 0x0, 1);

  return result;
}

bool PackagingMachineNode::ctrl_roller(
  const uint8_t days, 
  const bool home, 
  const bool ctrl
)
{
  bool result = true;

  if (!ctrl) 
  {
    result &= call_co_write(0x6039, 0x0, 0);
    return result;
  }

  if (home) 
  {
    result &= call_co_write(0x6030, 0x0, 1);
    result &= call_co_write(0x6037, 0x0, 1); // set mode 1 to go home
  }  else {
    result &= call_co_write(0x6030, 0x0, days > 7 ? 7 : days);
    result &= call_co_write(0x6037, 0x0, 0); // set mode 0 to go X day(s)
  }

  result &= call_co_write(0x6032, 0x0, 0); // direction must be 0 
  result &= call_co_write(0x6039, 0x0, 1);

  return result;
}

bool PackagingMachineNode::ctrl_conveyor(
  const uint16_t speed, 
  const bool stop_by_ph, 
  const bool fwd, 
  const bool ctrl)
{
  bool result = true;

  if (!ctrl) 
  {
    result &= call_co_write(0x6089, 0x0, 0);
    return result;
  }

  result &= call_co_write(0x6080, 0x0, speed > 3000 ? 3000 : speed);
  result &= call_co_write(0x6081, 0x0, stop_by_ph ? 1 : 0);
  result &= call_co_write(0x6082, 0x0, fwd ? 0 : 1);
  result &= call_co_write(0x6089, 0x0, 1);
  
  return result;
}

bool PackagingMachineNode::ctrl_pkg_len(
  const uint8_t level, 
  const bool ctrl)
{
  bool result = true;

  if (!ctrl) 
  {
    result &= call_co_write(0x6049, 0x0, 0);
    return result;
  }

  result &= call_co_write(0x6040, 0x0, 1); // move 1 step

  switch (level)
  {
  case 1:
    result &= call_co_write(0x6042, 0x0, 0); // moving downward
    break;
  case 2:
    result &= call_co_write(0x6042, 0x0, 1); // moving upward
    break;
  }

  result &= call_co_write(0x6049, 0x0, 1);
  
  return result;
}

void PackagingMachineNode::init_printer_config()
{
  printer_->configure(printer_config_->endpoint_in, printer_config_->endpoint_out, printer_config_->timeout);
  printer_->addDefaultConfig("SIZE", "75 mm,80 mm");
  printer_->addDefaultConfig("DIRECTION", "0, 0");
  printer_->addDefaultConfig("REFERENCE", std::to_string(printer_config_->dots_per_mm * 10) + ", " + std::to_string(0));
  printer_->addDefaultConfig("DENSITY", "2");
  printer_->addDefaultConfig("SPEED", "1");
  // printer_->addDefaultConfig("OFFSET", "4 mm");
  printer_->addDefaultConfig("CLS");
}

// TODO
// std::vector<std::string> PackagingMachineNode::get_print_label_cmd()
std::vector<std::string> PackagingMachineNode::get_print_label_cmd(std::string name, int total, int current)
{
  std::vector<std::string> cmds{};

  // auto num = generateRandomNumber(1000000000000000, 9999999999999999);
  auto num = 67642550;
  // auto timeslot = time_slot[current % time_slot.size()];

  cmds.emplace_back(
      R"(TEXT )" + std::to_string(0) + "," + std::to_string(50) + R"(,"4",0,1,1,")" + name + R"(")"
  );
  cmds.emplace_back(
      R"(BARCODE )" + std::to_string(350 + 0) + "," + std::to_string(100 + 0) + R"(,"128",128,1,0,2,4,")" + std::to_string(num) + R"(")"
  );
  // cmds.emplace_back(
  //     R"(TEXT )" + std::to_string(50 + offset_x) + "," + std::to_string(110 + offset_y) + R"(,"3",0,1,1,"No. )" + std::to_string(1 + current) + R"(")"
  // );
  // cmds.emplace_back(
  //     R"(TEXT )" + std::to_string(50 + offset_x) + "," + std::to_string(160 + offset_y) + R"(,"3",0,1,1,")" + "01-07-2024" + R"(")"
  // );
  // cmds.emplace_back(
  //     R"(TEXT )" + std::to_string(50 + offset_x) + "," + std::to_string(210 + offset_y) + R"(,"3",0,1,1,")" + std::to_string(timeslot) + R"(")"
  // );
  // cmds.emplace_back(
  //     R"(TEXT )" + std::to_string(0 + offset_x) + "," + std::to_string(300 + offset_y) + R"(,"3",0,1,1,")" + "MIRACID 20 mg capsule" + R"(")"
  // );
  cmds.emplace_back(
      R"(QRCODE )" + std::to_string(450 + 0) + "," + std::to_string(280 + 0) + R"(,L,5,A,0,")" + "SAM" + R"(")"
  );
  // cmds.emplace_back(
  //     R"(TEXT )" + std::to_string(50 + offset_x) + "," + std::to_string(450 + offset_y) + R"(,"4",0,1,1,")" + std::to_string(1 + current) + "/" + std::to_string(total) + R"(")"
  // );
  cmds.emplace_back("PRINT 1");

  return cmds;
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
    if (!ctrl_conveyor(300, 1, 1, 1))
    {
      status_->packaging_machine_state = PackagingMachineStatus::ERROR;
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  // TODO: read the photoelectic sensor state to make sure the material box is stopped
  uint8_t retry = 0;
  const uint8_t MAX_RETRY = 10;
  rclcpp::Rate loop_rate(500ms); 
  for (; retry < MAX_RETRY && rclcpp::ok(); ++retry) 
  {
    if (info_->conveyor) 
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
  ctrl_material_box_gate(1);
  std::this_thread::sleep_for(1s);
  ctrl_material_box_gate(0);

  are_drugs_fallen = true;
  goal_handle->publish_feedback(feedback);

  {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    status_->conveyor_state = PackagingMachineStatus::AVAILABLE;
  }

  UnbindRequest msg;
  msg.packaging_machine_id = status_->packaging_machine_id;
  msg.order_id = goal->order_id;
  msg.material_box_id = goal->material_box_id;
  unbind_req_publisher_->publish(msg);

  // TODO: packaging sequence 2
  uint8_t day = 0;
  uint8_t cell_index = 0;
  uint8_t print_index = 0;
  std::vector<uint8_t> to_be_print;

  for (uint8_t i = 0; i < CELLS; i++)
  {
    if (!goal->print_info[i].empty())
    {
      to_be_print.push_back(i);
    }
  }

  for (; print_index < PKG_PREFIX; print_index++)
  {
    if (to_be_print.at(print_index))
    {
      // print to_be_print.at(print_index)
    } else {
      // print empty
    }
    ctrl_pkg_dis(status_->package_length, 1, 1);
    wait_for_idle_motor(motor_status_->pkg_dis_state, 200);
    ctrl_squeezer(1, 1);
    wait_for_idle_motor(motor_status_->squ_state, 200);
    ctrl_squeezer(0, 1);
    wait_for_idle_motor(motor_status_->squ_state, 200);
  }

  for (; day < DAYS; day++)
  {
    ctrl_roller(1, 0, 1);
    wait_for_idle_motor(motor_status_->roller_state, 200);
    for (uint8_t i = 0; i < CELLS_PER_DAY; i++)
    {
      RCLCPP_INFO(this->get_logger(), "cell_index: %d", cell_index);
      ctrl_pill_gate(PILL_GATE_WIDTH, 1, 1);
      wait_for_idle_motor(motor_status_->pill_gate_state, 200);

      auto it = std::find(to_be_print.begin(), to_be_print.end(), cell_index);
      
      if (it != to_be_print.end())
      {
        if (to_be_print.at(print_index))
        {
          // print to_be_print.at(print_index)
        } else {
          // print empty
        }
        ctrl_pkg_dis(status_->package_length, 1, 1);
        wait_for_idle_motor(motor_status_->pkg_dis_state, 200);
        ctrl_squeezer(1, 1);
        wait_for_idle_motor(motor_status_->squ_state, 200);
        ctrl_squeezer(0, 1);
        wait_for_idle_motor(motor_status_->squ_state, 200);
        print_index++;
      }
      curr_order_status[cell_index] = true;
      goal_handle->publish_feedback(feedback);
      cell_index++;
    }
    ctrl_pill_gate(PILL_GATE_WIDTH * 4 * 1.05, 0, 1);
    wait_for_idle_motor(motor_status_->pill_gate_state, 200);
  }
  ctrl_roller(0, 1, 1);
  wait_for_idle_motor(motor_status_->roller_state, 200);

  for (uint8_t i = 0; i < PKG_PREFIX; i++)
  {
    // print empty
  }

  ctrl_cutter(1);
  std::this_thread::sleep_for(1s);
  ctrl_cutter(0);

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