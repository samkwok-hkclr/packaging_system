#include "packaging_machine_control_system/packaging_machine_node.hpp"

PackagingMachineNode::PackagingMachineNode(const rclcpp::NodeOptions& options)
: Node("packaging_machine_node", options)
{
  status_ = std::make_shared<PackagingMachineStatus>();
  motor_status_ = std::make_shared<MotorStatus>();
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

  status_->header.frame_id = "Packaging Machine";
  status_->packaging_machine_state = PackagingMachineStatus::IDLE;

  status_->package_length = 80; // FIXME

  info_->rs_state.reserve(NO_OF_REED_SWITCHS);

  srv_cli_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_ser_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rpdo_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions rpdo_options;
  rpdo_options.callback_group = rpdo_cbg_;
  // rclcpp_action::Options action_ser_options;
  // action_ser_options.callback_group = action_ser_cbg_;

  status_timer_ = this->create_wall_timer(1s, std::bind(&PackagingMachineNode::pub_status_cb, this), rpdo_cbg_);
  // add a "/" prefix to topic name avoid adding a namespace
  status_publisher_ = this->create_publisher<PackagingMachineStatus>("/machine_status", 10); 
  motor_status_publisher_ = this->create_publisher<MotorStatus>("motor_status", 10); 

  tpdo_pub_ = this->create_publisher<COData>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/tpdo", 
    10);
  rpdo_sub_ = this->create_subscription<COData>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/rpdo", 
    10,
    std::bind(&PackagingMachineNode::rpdo_cb, this, std::placeholders::_1),
    rpdo_options);
  
  co_read_client_ = this->create_client<CORead>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/sdo_read",
    rmw_qos_profile_services_default,
    srv_cli_cbg_);
  co_write_client_ = this->create_client<COWrite>(
    "/packaging_machine_" + std::to_string(status_->packaging_machine_id) + "/sdo_write",
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  init_package_machine_ = this->create_service<Trigger>(
    "init_package_machine", 
    std::bind(&PackagingMachineNode::init_handle, 
      this, 
      std::placeholders::_1, 
      std::placeholders::_2));
    
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
    co_read_wait_for_service();
    co_write_wait_for_service();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The CO Service client is up.");
  }
}

void PackagingMachineNode::init_packaging_machine(void)
{
  RCLCPP_INFO(this->get_logger(), "init_packaging_machine start");
  ctrl_heater(1);
  std::this_thread::sleep_for(GENERAL_STEP_DELAY);

  ctrl_stopper(1);
  wait_for_stopper(0);
  
  ctrl_material_box_gate(1);
  wait_for_material_box_gate(1);

  std::this_thread::sleep_for(GENERAL_VALVE_DELAY);

  ctrl_stopper(0);
  wait_for_stopper(1);

  ctrl_material_box_gate(0);
  wait_for_material_box_gate(0);

  std::this_thread::sleep_for(GENERAL_STEP_DELAY);

  for (uint8_t i = 0; i < CELLS_PER_DAY; i++)
  {
    ctrl_pill_gate(PILL_GATE_WIDTH, 1, 1);
    wait_for_pill_gate(MotorStatus::IDLE);

    std::this_thread::sleep_for(GENERAL_STEP_DELAY);
  }

  ctrl_pill_gate(PILL_GATE_WIDTH * 4 * PILL_GATE_CLOSE_MARGIN, 0, 1);
  wait_for_pill_gate(MotorStatus::IDLE);

  std::this_thread::sleep_for(GENERAL_STEP_DELAY);

  printer_ = std::make_shared<Printer>(
    printer_config_->vendor_id, 
    printer_config_->product_id, 
    printer_config_->serial);
  RCLCPP_INFO(this->get_logger(), "printer initialized");
  init_printer_config();

  for (uint8_t i = 0; i < 4; i++)
  {
    PackageInfo __msg;
    __msg.cn_name = "TESTING";
    __msg.en_name = "TESTING";
    __msg.date = "2024-11-11";
    __msg.time = "13:00";
    __msg.drugs.push_back("DRUG 1");
    auto cmd = get_print_label_cmd(__msg);
    printer_->runTask(cmd);
    RCLCPP_INFO(this->get_logger(), "printed a empty package");

    std::this_thread::sleep_for(PKG_DIS_WAIT_FOR_PRINTER_DELAY);
    ctrl_pkg_dis(status_->package_length * PKG_DIS_MARGIN, 1, 1);
    wait_for_pkg_dis(MotorStatus::IDLE);

    std::this_thread::sleep_for(PKG_DIS_BEFORE_SQUEEZER_DELAY);

    ctrl_squeezer(1, 1);
    wait_for_squeezer(MotorStatus::IDLE);

    std::this_thread::sleep_for(SQUEEZER_DELAY);

    ctrl_squeezer(0, 1);
    wait_for_squeezer(MotorStatus::IDLE);
  }

  printer_.reset();
  RCLCPP_INFO(this->get_logger(), "printer destroyed");

  ctrl_conveyor(CONVEYOR_SPEED, 0, 1, 1);
  std::this_thread::sleep_for(CONVEYOR_TESTING_DELAY);
  ctrl_conveyor(CONVEYOR_SPEED, 0, 1, 0);

  ctrl_roller(0, 1, 1);
  wait_for_roller(MotorStatus::IDLE);

  const std::lock_guard<std::mutex> lock(this->mutex_);
  status_->conveyor_state = PackagingMachineStatus::AVAILABLE;

  RCLCPP_INFO(this->get_logger(), "init_packaging_machine end");
}

void PackagingMachineNode::pub_status_cb(void)
{
  const std::lock_guard<std::mutex> lock(this->mutex_);
  status_->header.stamp = this->get_clock()->now();
  status_publisher_->publish(*status_);
  motor_status_publisher_->publish(*motor_status_);
}

inline void PackagingMachineNode::co_write_wait_for_service(void)
{
  while (!co_write_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting");
      rclcpp::shutdown();
    }
    RCLCPP_ERROR(this->get_logger(), "COWrite Service not available, waiting again...");
  }
}

inline void PackagingMachineNode::co_read_wait_for_service(void)
{
  while (!co_read_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting");
      rclcpp::shutdown();
    }
    RCLCPP_ERROR(this->get_logger(), "CORead Service not available, waiting again...");
  }
}

bool PackagingMachineNode::call_co_write(uint16_t index, uint8_t subindex, uint32_t data)
{
  std::shared_ptr<COWrite::Request> request = std::make_shared<COWrite::Request>();

  request->index = index;
  request->subindex = subindex;
  request->data = data;

  co_write_wait_for_service();

  using ServiceResponseFuture = rclcpp::Client<COWrite>::SharedFuture;
  auto response_received_cb = [this](ServiceResponseFuture future) {
    auto srv_response = future.get();
    if (srv_response) 
      RCLCPP_DEBUG(this->get_logger(), "Inside the COWrite Callback OK");
    else 
      RCLCPP_ERROR(this->get_logger(), "Inside the COWrite Callback NOT OK");
  };

  auto future = co_write_client_->async_send_request(request, response_received_cb);

  std::future_status status = future.wait_for(500ms);
  switch (status)
  {
  case std::future_status::ready:
    RCLCPP_DEBUG(this->get_logger(), "call_co_write wait_for OK");
    return true; 
  case std::future_status::timeout:
    RCLCPP_ERROR(this->get_logger(), "call_co_write wait_for timeout");
    return false;
  default: 
    RCLCPP_ERROR(this->get_logger(), "call_co_write wait_for NOT OK");
    return false;
  }
}

bool PackagingMachineNode::call_co_read(uint16_t index, uint8_t subindex, std::shared_ptr<uint32_t> data)
{
  std::shared_ptr<CORead::Request> request = std::make_shared<CORead::Request>();

  request->index = index;
  request->subindex = subindex;

  co_read_wait_for_service();

  using ServiceResponseFuture = rclcpp::Client<CORead>::SharedFuture;
  auto response_received_cb = [this, data](ServiceResponseFuture future) {
    auto srv_response = future.get();
    if (srv_response) 
    {
      *data = srv_response->data;
      RCLCPP_DEBUG(this->get_logger(), "Inside the COWrite Callback OK");
    } else 
    {
      RCLCPP_ERROR(this->get_logger(), "Inside the COWrite Callback NOT OK");
    }
  };
 
  auto future = co_read_client_->async_send_request(request, response_received_cb);

  std::future_status status = future.wait_for(500ms);
  switch (status)
  {
  case std::future_status::ready:
    RCLCPP_DEBUG(this->get_logger(), "call_co_read wait_for OK");
    return true;
  case std::future_status::timeout:
    RCLCPP_ERROR(this->get_logger(), "call_co_read wait_for timeout");
    return false;
  default: 
    RCLCPP_ERROR(this->get_logger(), "call_co_read wait_for NOT OK");
    return false;
  }
}

bool PackagingMachineNode::call_co_write_w_spin(uint16_t index, uint8_t subindex, uint32_t data)
{
  std::shared_ptr<COWrite::Request> request = std::make_shared<COWrite::Request>();

  request->index = index;
  request->subindex = subindex;
  request->data = data;

  co_write_wait_for_service();

  auto future = co_write_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 200ms) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    return response->success;
  } else {
    return false;
  }
}

bool PackagingMachineNode::call_co_read_w_spin(uint16_t index, uint8_t subindex, std::shared_ptr<uint32_t> data)
{
  std::shared_ptr<CORead::Request> request = std::make_shared<CORead::Request>();

  request->index = index;
  request->subindex = subindex;

  co_read_wait_for_service();
 
  auto future = co_read_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 200ms) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    *data = response->data;
    return response->success;
  } else {
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
    RCLCPP_DEBUG(this->get_logger(), "stopper: %s", info_->stopper ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "material_box_gate: %s", info_->material_box_gate ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "cutter: %s", info_->cutter ? "1" : "0");
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
    info_->conveyor        = input & 0x1;
    info_->squeeze         = (input >> 1) & 0x1;
    info_->squeeze_home    = (input >> 2) & 0x1;
    info_->roller_step     = (input >> 3) & 0x1;
    info_->roller_home     = (input >> 4) & 0x1;
    info_->pill_gate_home  = (input >> 5) & 0x1;
    info_->pkg_len_level_1 = (input >> 6) & 0x1;
    info_->pkg_len_level_2 = (input >> 7) & 0x1;
    RCLCPP_DEBUG(this->get_logger(), "conveyor: %s", info_->conveyor ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "squeeze: %s", info_->squeeze ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "squeeze_home: %s", info_->squeeze_home ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "roller_step: %s", info_->roller_step ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "roller_home: %s", info_->roller_home ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "pill_gate_home: %s", info_->pill_gate_home ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "pkg_len_level_1: %s", info_->pkg_len_level_1 ? "1" : "0");
    RCLCPP_DEBUG(this->get_logger(), "pkg_len_level_2: %s", info_->pkg_len_level_2 ? "1" : "0");
    break;
  }
  }
}

// ===================================== Service =====================================
void PackagingMachineNode::init_handle(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void) request;
  init_packaging_machine();
  response->success = true;
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
    response->message = "Error to control the heater";
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
    response->message = "Error to control the stopper";
  }
}

void PackagingMachineNode::conveyor_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  if (ctrl_conveyor(CONVEYOR_SPEED, 0, 1, request->data))
    response->success = true;
  else
  {
    response->success = false;
    response->message = "Error to control the conveyor";
  }
}

// ===================================== heater =====================================
bool PackagingMachineNode::ctrl_heater(const bool on)
{
  bool result = write_heater(on ? 1 : 0);
  if (result)
    RCLCPP_INFO(this->get_logger(), "%s the heater", on ? "turn-on" : "turn-off");
  return result;
}

inline bool PackagingMachineNode::write_heater(const uint32_t data)
{
  return call_co_write(0x6003, 0x0, data);
}

inline bool PackagingMachineNode::read_heater(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6003, 0x0, data);
}

// ===================================== stopper =====================================
bool PackagingMachineNode::ctrl_stopper(const bool protrude)
{
  bool result = write_stopper(protrude ? 0 : 1);
  if (result)
    RCLCPP_INFO(this->get_logger(), "%s the stopper", protrude ? "protrude" : "sunk");
  return result;
}

inline bool PackagingMachineNode::write_stopper(const uint32_t data)
{
  return call_co_write(0x6050, 0x0, data);
}

inline bool PackagingMachineNode::read_stopper(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6054, 0x0, data);
}

// ===================================== material_box_gate =====================================
bool PackagingMachineNode::ctrl_material_box_gate(const bool open)
{
  bool result = write_material_box_gate(open ? 1 : 0);
  if (result)
    RCLCPP_INFO(this->get_logger(), "%s the material box gate", open ? "Open" : "Close");
  return result;
}

inline bool PackagingMachineNode::write_material_box_gate(const uint32_t data)
{
  return call_co_write(0x6051, 0x0, data);
}

inline bool PackagingMachineNode::read_material_box_gate(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6055, 0x0, data);
}

// ===================================== cutter =====================================
bool PackagingMachineNode::ctrl_cutter(const bool cut)
{
  bool result = write_cutter(cut ? 1 : 0);
  if (result)
    RCLCPP_INFO(this->get_logger(), "%s the cutter", cut ? "Switch-on" : "Switch-off");
  return result;
}

inline bool PackagingMachineNode::write_cutter(const uint32_t data)
{
  return call_co_write(0x6052, 0x0, data);
}

inline bool PackagingMachineNode::read_cutter(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6056, 0x0, data);
}

// ===================================== pkg_dis =====================================
bool PackagingMachineNode::ctrl_pkg_dis(
  const float length, 
  const bool feed, 
  const bool ctrl
)
{
  bool result = true;

  result &= call_co_write(0x6011, 0x0, static_cast<uint32_t>(PULSES_PER_REV * length / (2 * M_PI * PKG_DIS_RADIUS)));
  result &= call_co_write(0x6012, 0x0, feed ? 1 : 0); // Set to 0 to feed the package out
  result &= call_co_write(0x6019, 0x0, ctrl ? 1 : 0);

  if (result)
    RCLCPP_INFO(this->get_logger(), "%s the package: %.2fmm", feed ? "feed" : "unfeed", length);
  return result;
}

inline bool PackagingMachineNode::read_pkg_dis_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6018, 0x0, data);
}

inline bool PackagingMachineNode::read_pkg_dis_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6019, 0x0, data);
}

// ===================================== pill_gate =====================================
bool PackagingMachineNode::ctrl_pill_gate(
  const float length, 
  const bool open, 
  const bool ctrl)
{
  bool result = true;

  result &= call_co_write(0x6021, 0x0, static_cast<uint32_t>(PULSES_PER_REV * length / (2 * M_PI * PILL_GATE_RADIUS)));
  result &= call_co_write(0x6022, 0x0, open ? 1 : 0);
  result &= call_co_write(0x6029, 0x0, ctrl ? 1 : 0);

  if (result)
    RCLCPP_INFO(this->get_logger(), "%s pill gate: %.2fmm", open ? "Open" : "Close", length);

  return result;
}

inline bool PackagingMachineNode::read_pill_gate_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6028, 0x0, data);
}

inline bool PackagingMachineNode::read_pill_gate_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6029, 0x0, data);
}

// ===================================== squeezer =====================================
bool PackagingMachineNode::ctrl_squeezer(
  const bool squeeze, 
  const bool ctrl)
{
  bool result = true;

  if (!ctrl) 
  {
    result &= call_co_write(0x6079, 0x0, 0);
    if (result)
      RCLCPP_INFO(this->get_logger(), "Stop the squeezer");

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

  if (result)
    RCLCPP_INFO(this->get_logger(), "%s the squeezer", squeeze ? "push" : "pull");

  return result;
}

inline bool PackagingMachineNode::read_squeezer_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6078, 0x0, data);
}

inline bool PackagingMachineNode::read_squeezer_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6079, 0x0, data);
}

// ===================================== conveyor =====================================
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
    if (result)
      RCLCPP_INFO(this->get_logger(), "Stop the conveyor");

    return result;
  }

  result &= call_co_write(0x6080, 0x0, speed > 3000 ? 3000 : speed);
  result &= call_co_write(0x6081, 0x0, stop_by_ph ? 1 : 0);
  result &= call_co_write(0x6082, 0x0, fwd ? 0 : 1);
  result &= call_co_write(0x6089, 0x0, 1);

  if (result)
    RCLCPP_INFO(this->get_logger(), "moving the conveyor %s", stop_by_ph ? "with stop by photoelectric sensor" : "");
  
  return result;
}

inline bool PackagingMachineNode::read_conveyor_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6088, 0x0, data);
}

inline bool PackagingMachineNode::read_conveyor_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6089, 0x0, data);
}

// ===================================== roller =====================================
bool PackagingMachineNode::ctrl_roller(
  const uint8_t days, 
  const bool home, 
  const bool ctrl)
{
  bool result = true;

  if (!ctrl) 
  {
    result &= call_co_write(0x6039, 0x0, 0);
    if (result)
      RCLCPP_INFO(this->get_logger(), "Stop the roller");

    return result;
  }

  if (home) 
  {
    result &= call_co_write(0x6030, 0x0, 1); // must be 1 step
    result &= call_co_write(0x6037, 0x0, 1); // set mode 1 to go home
  }  else {
    result &= call_co_write(0x6030, 0x0, days > DAYS ? DAYS : days);
    result &= call_co_write(0x6037, 0x0, 0); // set mode 0 to go X day(s)
  }

  result &= call_co_write(0x6032, 0x0, 0); // direction must be 0 
  result &= call_co_write(0x6039, 0x0, 1);

  if (result)
  {
    if (home)
      RCLCPP_INFO(this->get_logger(), "moving the roller to home");
    else
      RCLCPP_INFO(this->get_logger(), "moving the roller to %s day(s)", std::to_string(days).c_str());
  }

  return result;
}

inline bool PackagingMachineNode::read_roller_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6038, 0x0, data);
}

inline bool PackagingMachineNode::read_roller_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6039, 0x0, data);
}

// ===================================== pkg_len =====================================
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
  default:
    return ctrl_pkg_len(0, 0);
    break;
  }

  result &= call_co_write(0x6049, 0x0, 1);

  if (result)
    RCLCPP_INFO(this->get_logger(), "moving the pkg len to level ???"); // FIXME
  
  return result;
}

inline bool PackagingMachineNode::read_pkg_len_state(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6048, 0x0, data);
}

inline bool PackagingMachineNode::read_pkg_len_ctrl(std::shared_ptr<uint32_t> data)
{
  return call_co_read(0x6049, 0x0, data);
}

// ===================================== wait for =====================================
void PackagingMachineNode::wait_for_stopper(const uint32_t stop_condition)
{
  std::this_thread::sleep_for(200ms); 

  rclcpp::Rate rate(5);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);
    read_stopper(data);
    RCLCPP_DEBUG(this->get_logger(), "stopper: %d", *data);

    if (*data == stop_condition)
      break;

    if (!rclcpp::ok())
    {
      RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the stopper. Exiting");
      break; 
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_material_box_gate(const uint32_t stop_condition)
{
  std::this_thread::sleep_for(200ms); 

  rclcpp::Rate rate(5);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);
    read_material_box_gate(data);
    RCLCPP_DEBUG(this->get_logger(), "material_box_gate: %d", *data);

    if (*data == stop_condition)
      break;

    if (!rclcpp::ok())
    {
      RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the material_box_gate. Exiting");
      break; 
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_cutter(const uint32_t stop_condition)
{
  std::this_thread::sleep_for(200ms); 

  rclcpp::Rate rate(5);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> data = std::make_shared<uint32_t>(0);
    read_cutter(data);
    RCLCPP_DEBUG(this->get_logger(), "cutter: %d", *data);

    if (*data == stop_condition)
      break;

    if (!rclcpp::ok())
    {
      RCLCPP_WARN(this->get_logger(), "Interrupted while waiting for the cutter. Exiting");
      break; 
    }

    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_pkg_dis(const uint8_t target_state)
{
  std::this_thread::sleep_for(200ms); 

  rclcpp::Rate rate(5);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    read_pkg_dis_state(state);
    read_pkg_dis_ctrl(ctrl);
    RCLCPP_DEBUG(this->get_logger(), "pkg_dis_state: %d, ctrl: %d", *state, *ctrl);

    {
      const std::lock_guard<std::mutex> lock(this->mutex_);
      motor_status_->pkg_dis_state = *state;
      if (motor_status_->pkg_dis_state == target_state && *ctrl == 0)
      {
        RCLCPP_INFO(this->get_logger(), "pkg_dis is idle");
        break;
      }
    }
    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_pill_gate(const uint8_t target_state)
{
  std::this_thread::sleep_for(200ms); 

  rclcpp::Rate rate(5);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    read_pill_gate_state(state);
    read_pill_gate_ctrl(ctrl);
    RCLCPP_DEBUG(this->get_logger(), "pill_gate_state: %d, ctrl: %d", *state, *ctrl);

    {
      const std::lock_guard<std::mutex> lock(this->mutex_);
      motor_status_->pill_gate_state = *state;
      if (motor_status_->pill_gate_state == target_state && *ctrl == 0)
      {
        RCLCPP_INFO(this->get_logger(), "pill_gate is idle");
        break;
      }
    }
    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_squeezer(const uint8_t target_state)
{
  std::this_thread::sleep_for(400ms); 

  rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    read_squeezer_state(state);
    read_squeezer_ctrl(ctrl);
    RCLCPP_DEBUG(this->get_logger(), "squeezer_state: %d, ctrl: %d", *state, *ctrl);

    {
      const std::lock_guard<std::mutex> lock(this->mutex_);
      motor_status_->squ_state = *state;
      if (motor_status_->squ_state == target_state && *ctrl == 0)
      {
        RCLCPP_INFO(this->get_logger(), "squeezer_state is idle");
        break;
      }
    }
    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_conveyor(const uint8_t target_state)
{
  std::this_thread::sleep_for(400ms); 

  rclcpp::Rate rate(2);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    read_conveyor_state(state);
    read_conveyor_ctrl(ctrl);
    RCLCPP_DEBUG(this->get_logger(), "conveyor_state: %d, ctrl: %d", *state, *ctrl);

    {
      const std::lock_guard<std::mutex> lock(this->mutex_);
      motor_status_->con_state = *state;
      if (motor_status_->con_state == target_state && *ctrl == 0)
      {
        RCLCPP_INFO(this->get_logger(), "conveyor_state is idle");
        break;
      }
    }
    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_roller(const uint8_t target_state)
{
  std::this_thread::sleep_for(200ms); 

  rclcpp::Rate rate(5);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    read_roller_state(state);
    read_roller_ctrl(ctrl);
    RCLCPP_DEBUG(this->get_logger(), "roller_state: %d, ctrl: %d", *state, *ctrl);

    {
      const std::lock_guard<std::mutex> lock(this->mutex_);
      motor_status_->roller_state = *state;
      if (motor_status_->roller_state == target_state && *ctrl == 0)
      {
        RCLCPP_INFO(this->get_logger(), "roller_state is idle");
        break;
      }
    }
    rate.sleep();
  }
}

void PackagingMachineNode::wait_for_pkg_len(const uint8_t target_state)
{
  std::this_thread::sleep_for(200ms); 

  rclcpp::Rate rate(5);
  while (rclcpp::ok())
  {
    std::shared_ptr<uint32_t> state = std::make_shared<uint32_t>(0);
    std::shared_ptr<uint32_t> ctrl = std::make_shared<uint32_t>(0);
    read_pkg_len_state(state);
    read_pkg_len_ctrl(ctrl);
    RCLCPP_DEBUG(this->get_logger(), "pkg_len_state: %d, ctrl: %d", *state, *ctrl);

    {
      const std::lock_guard<std::mutex> lock(this->mutex_);
      motor_status_->pkg_len_state = *state;
      if (motor_status_->pkg_len_state == target_state && *ctrl == 0)
      {
        RCLCPP_INFO(this->get_logger(), "pkg_len_state is idle");
        break;
      }
    }
    rate.sleep();
  }
}

// ===================================== printer =====================================
void PackagingMachineNode::init_printer_config()
{
  printer_->configure(printer_config_->endpoint_in, printer_config_->endpoint_out, printer_config_->timeout);
  printer_->addDefaultConfig("SIZE", "75 mm,80 mm");
  printer_->addDefaultConfig("GAP", "0 mm, 0mm");
  printer_->addDefaultConfig("DIRECTION", "0, 0");
  printer_->addDefaultConfig("REFERENCE", "-90, -120");
  printer_->addDefaultConfig("OFFSET", "0 mm");
  printer_->addDefaultConfig("SHIFT", "0");
  printer_->addDefaultConfig("SET", "TEAR OFF");
  printer_->addDefaultConfig("SET", "REWIND OFF");
  printer_->addDefaultConfig("SET", "PEEL OFF");
  printer_->addDefaultConfig("SET", "CUTTER OFF");
  printer_->addDefaultConfig("SET", "PARTIAL_CUTTER OFF");
  printer_->addDefaultConfig("CLS");
  // printer_->addDefaultConfig("DENSITY", "8");
  // printer_->addDefaultConfig("SPEED", "1");
  // printer_->addDefaultConfig("REFERENCE", std::to_string(printer_config_->dots_per_mm * 10) + ", " + std::to_string(0));
}

std::vector<std::string> PackagingMachineNode::get_print_label_cmd(PackageInfo msg)
{
  std::vector<std::string> cmds{};
  if (!msg.en_name.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Add a english name: %s", msg.en_name.c_str());

    std::string gbk_cn = printer_->convert_utf8_to_gbk(msg.cn_name);
    std::string cn = "TEXT 240,180,\"TSS24.BF2\",0,2,2,\"" + gbk_cn + "\"";
    cmds.emplace_back(cn);
    std::string en = "TEXT 600,186,\"TSS24.BF2\",0,2,2,\"" + msg.en_name + "\"";
    cmds.emplace_back(en);
    std::string d = "TEXT 240,270,\"4\",0,1,1,\"" + msg.date + "\"";
    cmds.emplace_back(d);
    std::string t = "TEXT 240,334,\"4\",0,1,1,\"" + msg.time + "\"";
    cmds.emplace_back(t);
    cmds.emplace_back("QRCODE 684,252,L,6,A,0,\"www.hkclr.hk\"");
    for (size_t index = 0; index < msg.drugs.size(); ++index) 
    {
      std::string utf_md = msg.drugs[index];
      std::string gbk_md = printer_->convert_utf8_to_gbk(utf_md);
      int y = 400 + index * 64;
      std::string y_label = std::to_string(y);
      std::string m = "TEXT 240,"+ y_label + ",\"TSS24.BF2\",0,2,2,\"" + gbk_md + "\"";
      cmds.emplace_back(m);
    }
  }

  cmds.emplace_back("PRINT 1,1");
  RCLCPP_DEBUG(this->get_logger(), "printer commands are ready");
  return cmds;
}

rclcpp_action::GoalResponse PackagingMachineNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PackagingOrder::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "print_info size: %lu", goal->print_info.size());

  printer_.reset();
  printer_ = std::make_shared<Printer>(
    printer_config_->vendor_id, 
    printer_config_->product_id, 
    printer_config_->serial);
  RCLCPP_INFO(this->get_logger(), "printer initialized");
  init_printer_config();
  
  {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    status_->packaging_machine_state = PackagingMachineStatus::BLOCKING;
    status_->conveyor_state = PackagingMachineStatus::UNAVAILABLE;
    RCLCPP_INFO(this->get_logger(), "set packaging_machine_state to BLOCKING");
    RCLCPP_INFO(this->get_logger(), "set conveyor_state to UNAVAILABLE");
  }
  
  ctrl_stopper(1);
  wait_for_stopper(0);

  if (!ctrl_conveyor(CONVEYOR_SPEED, 1, 1, 1))
  {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    status_->packaging_machine_state = PackagingMachineStatus::ERROR;
    RCLCPP_ERROR(this->get_logger(), "set packaging_machine_state to ERROR");
    RCLCPP_ERROR(this->get_logger(), "REJECT");
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  // TODO: read the photoelectic sensor state to make sure the material box is stopped
  uint8_t retry = 0;
  const uint8_t MAX_RETRY = 10;
  rclcpp::Rate loop_rate(1s); 
  for (; retry < MAX_RETRY && rclcpp::ok(); ++retry) 
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for the material box, conveyor photoelectic: %s", info_->conveyor ? "1" : "0");
    {
      if (!info_->conveyor) 
      {
        const std::lock_guard<std::mutex> lock(this->mutex_);
        RCLCPP_INFO(this->get_logger(), "Checking conveyor photoelectric senser: %s", info_->conveyor ? "1" : "0");
        status_->packaging_machine_state = PackagingMachineStatus::BUSY;
        break;
      }
    }
    loop_rate.sleep();
  }
  if (retry >= MAX_RETRY)
  {
    RCLCPP_INFO(this->get_logger(), "retry(%d) >= MAX_RETRY", retry);
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

  RCLCPP_INFO(this->get_logger(), "======== packaging sequence 1 ==========");

  ctrl_material_box_gate(1);
  wait_for_material_box_gate(1);

  std::this_thread::sleep_for(3s);

  ctrl_material_box_gate(0);
  wait_for_material_box_gate(0);

  are_drugs_fallen = true;
  RCLCPP_INFO(this->get_logger(), "Set are_drugs_fallen to True");
  goal_handle->publish_feedback(feedback);

  {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    status_->conveyor_state = PackagingMachineStatus::AVAILABLE;
    RCLCPP_INFO(this->get_logger(), "Set conveyor_state to AVAILABLE");
  }

  ctrl_conveyor(CONVEYOR_SPEED, 0, 1, 1);
  // TODO: publish msg to request another packaging machine conveyor to move

  RCLCPP_INFO(this->get_logger(), "========== packaging sequence 2 ==========");
  uint8_t day = 0;
  size_t cell_index = 0;
  size_t print_index = 0;
  std::vector<size_t> to_be_printed{};

  for (uint8_t i = 0; i < CELLS; i++)
  {
    if (!goal->print_info[i].en_name.empty())
    {
      to_be_printed.push_back(i);
    }
  }
  for (uint8_t i = 0; i < to_be_printed.size(); i++) 
  {
    RCLCPP_INFO(this->get_logger(), "to_be_printed[%d]: %ld", i, to_be_printed.at(i));
  }
  RCLCPP_INFO(this->get_logger(), "print_info size: %ld", to_be_printed.size());

  // make sure the package is tight
  ctrl_pkg_dis(status_->package_length / 4, 1, 1);
  wait_for_pkg_dis(MotorStatus::IDLE);

  for (; print_index < PKG_PREFIX; print_index++)
  {
    RCLCPP_INFO(this->get_logger(), ">>>>>>>>>> cell_index: %ld <<<<<<<<<<", cell_index);
    RCLCPP_INFO(this->get_logger(), ">>>>>>>>>> print_index: %ld <<<<<<<<<<", print_index);
    if (print_index < to_be_printed.size())
    {
      std::vector<std::string> cmd = get_print_label_cmd(goal->print_info[cell_index]);
      printer_->runTask(cmd);
      RCLCPP_INFO(this->get_logger(), "printed a order %ld package", cell_index);
    } else {
      PackageInfo __msg;
      std::vector<std::string> cmd = get_print_label_cmd(__msg);
      printer_->runTask(cmd);
      RCLCPP_INFO(this->get_logger(), "printed a empty package");
    }
    std::this_thread::sleep_for(PKG_DIS_WAIT_FOR_PRINTER_DELAY);
    ctrl_pkg_dis(status_->package_length * PKG_DIS_MARGIN, 1, 1);
    wait_for_pkg_dis(MotorStatus::IDLE);

    ctrl_squeezer(1, 1);
    wait_for_squeezer(MotorStatus::IDLE);

    std::this_thread::sleep_for(SQUEEZER_DELAY);

    ctrl_squeezer(0, 1);
    wait_for_squeezer(MotorStatus::IDLE);
    
    if (cell_index < CELLS)
      cell_index++;
  }
  RCLCPP_INFO(this->get_logger(), "Printed 4 prefix");

  // FIXME: the flow is incorrect
  for (; day < DAYS; day++)
  {
    RCLCPP_INFO(this->get_logger(), "@@@@@@@@@@ Day: %d @@@@@@@@@@", day);

    ctrl_roller(1, 0, 1);
    wait_for_roller(MotorStatus::IDLE);

    std::this_thread::sleep_for(GENERAL_STEP_DELAY);

    for (uint8_t k = 0; k < CELLS_PER_DAY; k++)
    {
      RCLCPP_INFO(this->get_logger(), ">>>>>>>>>> cell_index: %ld <<<<<<<<<<", cell_index);
      RCLCPP_INFO(this->get_logger(), ">>>>>>>>>> print_index: %ld <<<<<<<<<<", print_index);
      ctrl_pill_gate(PILL_GATE_WIDTH, 1, 1);
      wait_for_pill_gate(MotorStatus::IDLE);

      auto it = std::find(to_be_printed.begin(), to_be_printed.end(), cell_index);
      
      if (it != to_be_printed.end() && print_index < to_be_printed.size())
      {
        std::vector<std::string> cmd = get_print_label_cmd(goal->print_info[cell_index]);
        printer_->runTask(cmd);
        RCLCPP_INFO(this->get_logger(), "printed a order %ld package", cell_index);

        std::this_thread::sleep_for(PKG_DIS_WAIT_FOR_PRINTER_DELAY);
        ctrl_pkg_dis(status_->package_length * PKG_DIS_MARGIN, 1, 1);
        wait_for_pkg_dis(MotorStatus::IDLE);

        ctrl_squeezer(1, 1);
        wait_for_squeezer(MotorStatus::IDLE);

        std::this_thread::sleep_for(SQUEEZER_DELAY);

        ctrl_squeezer(0, 1);
        wait_for_squeezer(MotorStatus::IDLE);
        print_index++;
      } 
      else if (cell_index == CELLS) 
      {
        PackageInfo __msg;
        std::vector<std::string> cmd = get_print_label_cmd(__msg);
        printer_->runTask(cmd);
        RCLCPP_INFO(this->get_logger(), "printed a empty package");

        std::this_thread::sleep_for(PKG_DIS_WAIT_FOR_PRINTER_DELAY);
        ctrl_pkg_dis(status_->package_length * PKG_DIS_MARGIN, 1, 1);
        wait_for_pkg_dis(MotorStatus::IDLE);

        ctrl_squeezer(1, 1);
        wait_for_squeezer(MotorStatus::IDLE);

        std::this_thread::sleep_for(SQUEEZER_DELAY);

        ctrl_squeezer(0, 1);
        wait_for_squeezer(MotorStatus::IDLE);
      }
      
      if (cell_index < CELLS)
      {
        curr_order_status[cell_index] = true;
        cell_index++;
      }
      goal_handle->publish_feedback(feedback);
    }

    ctrl_pill_gate(PILL_GATE_WIDTH * 4 * PILL_GATE_CLOSE_MARGIN, 0, 1);
    wait_for_pill_gate(MotorStatus::IDLE);
  }
  RCLCPP_INFO(this->get_logger(), ">>>>>>>>>> completed 28 cells <<<<<<<<<<");

  for (uint8_t i = 0; i < PKG_PREFIX; i++)
  {
    PackageInfo __msg;
    std::vector<std::string> cmd = get_print_label_cmd(__msg);
    printer_->runTask(cmd);
    RCLCPP_INFO(this->get_logger(), "printed a empty package");

    std::this_thread::sleep_for(PKG_DIS_WAIT_FOR_PRINTER_DELAY);
    ctrl_pkg_dis(status_->package_length * PKG_DIS_MARGIN, 1, 1);
    wait_for_pkg_dis(MotorStatus::IDLE);

    ctrl_squeezer(1, 1);
    wait_for_squeezer(MotorStatus::IDLE);

    std::this_thread::sleep_for(SQUEEZER_DELAY);

    ctrl_squeezer(0, 1);
    wait_for_squeezer(MotorStatus::IDLE);
  }

  ctrl_roller(0, 1, 1);
  wait_for_roller(MotorStatus::IDLE);
  
  // ctrl_cutter(1);
  // std::this_thread::sleep_for(GENERAL_VALVE_DELAY);
  // ctrl_cutter(0);

  // Check if goal is done
  if (rclcpp::ok()) {
    printer_.reset();
    RCLCPP_INFO(this->get_logger(), "printer destroyed");
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
  
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<PackagingMachineNode>(options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
}