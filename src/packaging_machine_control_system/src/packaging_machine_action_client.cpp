#include "packaging_machine_control_system/packaging_machine_action_client.hpp"

namespace action_client
{

PackagingMachineActionClient::PackagingMachineActionClient(const rclcpp::NodeOptions & options)
: Node("action_client", options),
  goal_done_(false)
  // print_info_(CELLS)
{
  this->declare_parameter<std::uint8_t>("packaging_machine_id", 0);
  this->declare_parameter<std::int64_t>("order_id", 0);
  this->declare_parameter<std::int64_t>("material_box_id", 0);
  this->declare_parameter<std::vector<std::string>>("cn_name", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("en_name", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("date", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("time", std::vector<std::string>{});
  this->declare_parameter<std::vector<std::string>>("drugs", std::vector<std::string>{});

  std::vector<std::string> _cn_name;
  std::vector<std::string> _en_name;
  std::vector<std::string> _date;
  std::vector<std::string> _time;
  std::vector<std::string> _drugs;

  this->get_parameter("packaging_machine_id", packaging_machine_id_);
  this->get_parameter("order_id", order_id_);
  this->get_parameter("material_box_id", material_box_id_);
  this->get_parameter("cn_name", _cn_name);
  this->get_parameter("en_name", _en_name);
  this->get_parameter("date", _date);
  this->get_parameter("time", _time);
  this->get_parameter("drugs", _drugs);

  for (size_t i = 0; i < CELLS; i++)
  {
    print_info_[i].cn_name = _cn_name[i];
    print_info_[i].en_name = _en_name[i];
    print_info_[i].date = _date[i];
    print_info_[i].time = _time[i];
    auto split_string = [](const std::string& s, char delimiter) {
      std::vector<std::string> result;
      std::stringstream ss(s);
      std::string item;

      while (std::getline(ss, item, delimiter)) {
        result.push_back(item);
      }

      return result;
    };
    print_info_[i].drugs = split_string(_drugs[i], '#');
  }

  for (size_t i = 0; i < CELLS; i++)
  {
    RCLCPP_DEBUG(this->get_logger(), print_info_[i].cn_name.c_str());
    RCLCPP_DEBUG(this->get_logger(), print_info_[i].en_name.c_str());
    RCLCPP_DEBUG(this->get_logger(), print_info_[i].date.c_str());
    RCLCPP_DEBUG(this->get_logger(), print_info_[i].time.c_str());
    for (size_t j = 0; j < print_info_[i].drugs.size(); j++)
    {
      RCLCPP_DEBUG(this->get_logger(), print_info_[i].drugs[j].c_str());
    }
  }

  const std::string action_server = "/packaging_machine_" + std::to_string(packaging_machine_id_) + "/packaging_order";
  
  packaging_status_ = std::make_shared<PackagingStatus>();
  packaging_status_->packaging_machine_id = packaging_machine_id_;
  packaging_status_->order_id = order_id_;

  this->client_ptr_ = rclcpp_action::create_client<PackagingOrder>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    action_server);

  packaging_status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
  std::bind(&PackagingMachineActionClient::pub_status_cb, this));

  this->send_goal_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PackagingMachineActionClient::send_goal, this));

  result_pub_ = this->create_publisher<PackagingResult>("packaging_result", 10);
  packaging_status_pub_ = this->create_publisher<PackagingStatus>("packaging_status", 10);

  RCLCPP_INFO(this->get_logger(), "An Action Client Component is created.");
}

inline bool PackagingMachineActionClient::is_goal_done(void) const
{
  return this->goal_done_;
}

void PackagingMachineActionClient::pub_status_cb(void)
{
  const std::lock_guard<std::mutex> lock(this->mutex_);
  packaging_status_pub_->publish(*packaging_status_);
}

void PackagingMachineActionClient::send_goal(void)
{
  using namespace std::placeholders;

  this->send_goal_timer_->cancel();

  this->goal_done_ = false;

  if (!this->client_ptr_) 
  {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!this->client_ptr_->wait_for_action_server()) 
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    this->goal_done_ = true;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Sending goal 3");

  auto goal_msg = PackagingOrder::Goal();
  goal_msg.order_id = order_id_;
  goal_msg.material_box_id = material_box_id_;
  std::copy(print_info_.begin(), print_info_.end(), goal_msg.print_info.begin()); 

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
    const std::lock_guard<std::mutex> lock(this->mutex_);
    packaging_status_->server_accepted = true;
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void PackagingMachineActionClient::feedback_callback(
  GaolHandlerPackagingOrder::SharedPtr,
  const std::shared_ptr<const PackagingOrder::Feedback> feedback)
{
  const std::lock_guard<std::mutex> lock(this->mutex_);
  packaging_status_->are_drugs_fallen = feedback->are_drugs_fallen;
  packaging_status_->order_status = feedback->curr_order_status;

  RCLCPP_INFO(this->get_logger(), "A feedback received");
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

  {
    const std::lock_guard<std::mutex> lock(this->mutex_);
    packaging_status_->is_completed = true;
    packaging_status_pub_->publish(*packaging_status_);
  }
  
  RCLCPP_INFO(this->get_logger(), "Goal Done");

  PackagingResult result_msg;
  result_msg.success = this->goal_done_;
  result_msg.packaging_machine_id = packaging_machine_id_;
  result_msg.order_id = order_id_;
  result_msg.material_box_id = material_box_id_;
  result_pub_->publish(result_msg);
}

} // namespace action_client

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(action_client::PackagingMachineActionClient)