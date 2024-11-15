#include "packaging_machine_control_system/packaging_machine_action_client.hpp"

namespace action_client
{

PackagingMachineActionClient::PackagingMachineActionClient(const rclcpp::NodeOptions & options)
: Node("action_client", options),
  goal_done_(false)
{
  this->declare_parameter<std::uint8_t>("packaging_machine_id", 0);
  this->declare_parameter<std::int64_t>("order_id", 0);
  this->declare_parameter<std::int64_t>("material_box_id", 0);
  this->declare_parameter<std::vector<std::string>>("print_info", std::vector<std::string>{});

  this->get_parameter("packaging_machine_id", packaging_machine_id_);
  this->get_parameter("order_id", order_id_);
  this->get_parameter("material_box_id", material_box_id_);
  this->get_parameter("print_info", print_info_);

  std::string action_server =  "/packaging_machine_" + std::to_string(packaging_machine_id_) + "/packaging_order";
  
  this->client_ptr_ = rclcpp_action::create_client<PackagingOrder>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    action_server);

  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PackagingMachineActionClient::send_goal, this));

  result_pub_ = this->create_publisher<PackagingResult>("packaging_result", 10);

  RCLCPP_INFO(this->get_logger(), "An Action Client Component is created.");
}

bool PackagingMachineActionClient::is_goal_done() const
{
  return this->goal_done_;
}

void PackagingMachineActionClient::send_goal()
{
  using namespace std::placeholders;
  RCLCPP_INFO(this->get_logger(), "Sending goal 1");

  this->timer_->cancel();

  this->goal_done_ = false;

  if (!this->client_ptr_) 
  {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }
  
  RCLCPP_INFO(this->get_logger(), "Sending goal 2");

  if (!this->client_ptr_->wait_for_action_server()) 
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    this->goal_done_ = true;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Sending goal 3");

  auto goal_msg = PackagingOrder::Goal();
  goal_msg.order_id = 101;
  std::vector<std::string> _print_info(GRIDS);
  for (size_t i = 0; i < GRIDS; i++)
  {
    _print_info[i] = "print me";
  }
  goal_msg.print_info = _print_info;
  RCLCPP_INFO(this->get_logger(), "print_info size: %zu", goal_msg.print_info.size());
  

  auto send_goal_options = rclcpp_action::Client<PackagingOrder>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&PackagingMachineActionClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&PackagingMachineActionClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&PackagingMachineActionClient::result_callback, this, _1);

  auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(this->get_logger(), "Sent goal");
}

void PackagingMachineActionClient::goal_response_callback(const GaolHandlerPackagingOrder::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void PackagingMachineActionClient::feedback_callback(
  GaolHandlerPackagingOrder::SharedPtr,
  const std::shared_ptr<const PackagingOrder::Feedback> feedback)
{
  (void) feedback;
  // std::stringstream ss;
  // ss << "Next number in sequence received: ";
  // for (auto number : feedback->partial_sequence) {
  //   ss << number << " ";
  // }
  RCLCPP_INFO(this->get_logger(), "feedback");
}

void PackagingMachineActionClient::result_callback(const GaolHandlerPackagingOrder::WrappedResult & result)
{
  this->goal_done_ = true;

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  // std::stringstream ss;
  // ss << "Result received: ";
  // for (auto number : result.result->sequence) {
  //   ss << number << " ";
  // }
  RCLCPP_INFO(this->get_logger(), "Goal Done");

  PackagingResult result_msg;
  result_msg.success = true;
  result_msg.packaging_machine_id = packaging_machine_id_;
  result_msg.order_id = order_id_;
  result_msg.material_box_id = material_box_id_;
  result_pub_->publish(result_msg);
}

} // namespace action_client

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(action_client::PackagingMachineActionClient)