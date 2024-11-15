#include "packaging_machine_control_system/packaging_machine_node.hpp"

PackagingMachineNode::PackagingMachineNode(const rclcpp::NodeOptions& options)
: Node("packaging_machine_node", options)
{
  status_ = std::make_shared<PackagingMachineStatus>();

  this->declare_parameter<uint8_t>("packaging_machine_id", 0);
  this->get_parameter("packaging_machine_id", status_->packaging_machine_id);
  status_->header.frame_id = "Packaging Machine";
  status_->packaging_machine_state = PackagingMachineStatus::IDLE;

  status_timer_ = this->create_wall_timer(1s, std::bind(&PackagingMachineNode::pub_status_cb, this));
  status_publisher_ = this->create_publisher<PackagingMachineStatus>("/machine_status", 10); // add a "/" to topic name avoid namespace

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
}

void PackagingMachineNode::pub_status_cb(void)
{
  const std::lock_guard<std::mutex> lock(this->mutex_);
  status_->header.stamp = this->get_clock()->now();
  status_publisher_->publish(*status_);
}

// TODO
void PackagingMachineNode::conveyor_handle(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  (void) request;
  // call SDO write service: set movement

  // call SDO Read service: conveyor state

  // fake state
  bool conveyor_state = false;
  response->success = true;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ID [%d] is %s", 
    status_->packaging_machine_id, 
    conveyor_state ? "ON" : "OFF");
}

// TODO
bool PackagingMachineNode::material_box_gate(bool open)
{
  // call SDO write service: set solenoid valve
 
  return true;
}

// TODO
bool stopper_movement(bool protrude)
{
  // call SDO write service: set solenoid valve

  return true;
}

rclcpp_action::GoalResponse PackagingMachineNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PackagingOrder::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "print_info size: %lu", goal->print_info.size());
  if (goal->print_info.size() == GRIDS)
  {
    {
      const std::lock_guard<std::mutex> lock(this->mutex_);
      status_->packaging_machine_state = PackagingMachineStatus::BUSY;
    }
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %u", goal->order_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Received goal request with order %u: Order Length Error", goal->order_id);
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "handle_goal end");
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
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&PackagingMachineNode::order_execute, this, _1), goal_handle}.detach();
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