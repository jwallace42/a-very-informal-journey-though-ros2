#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "node_thread.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace the_node {
using namespace std::chrono_literals;

template<typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
    FutureT & future,
    WaitTimeT time_to_wait)
{
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {break;}
    status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class LifecycleClient
{
 public:
  LifecycleClient(rclcpp::Node::SharedPtr node, const std::string &node_to_control)
  {
    logger_ = node->get_logger();
    std::string get_state_srv = node_to_control + "/get_state";
    std::string change_state_srv = node_to_control + "/change_state";
    get_state_client_ = node->create_client<lifecycle_msgs::srv::GetState>(get_state_srv);
    change_state_client_ = node->create_client<lifecycle_msgs::srv::ChangeState>(change_state_srv);
    node_to_control_ = node_to_control;
  }

  unsigned int get_state(std::chrono::seconds timeout= 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!get_state_client_->wait_for_service(10s)) {
      RCLCPP_ERROR(
          logger_,
          "Service %s is not available.",
          get_state_client_->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the service request for asking the current
    // state of the lc_talker node.
    auto future_result = get_state_client_->async_send_request(request).future.share();

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, timeout);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(
          logger_, "Server time out while getting current state for node");
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (future_result.get()) {
      RCLCPP_INFO(
          logger_, "Node %s has current state: %s", node_to_control_.c_str(),
          future_result.get()->current_state.label.c_str());
      return future_result.get()->current_state.id;
    } else {
      RCLCPP_ERROR(
          logger_, "Failed to get current state for node");
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  bool
  change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!change_state_client_->wait_for_service(time_out)) {
      RCLCPP_ERROR(
          logger_,
          "Service %s is not available.",
          change_state_client_->get_service_name());
      return false;
    }

    // We send the request with the transition we want to invoke.
    auto future_result = change_state_client_->async_send_request(request).future.share();

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(
          logger_, "Server time out while getting current state for node");
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success) {
      RCLCPP_INFO(
          logger_, "Transition %d successfully triggered.", static_cast<int>(transition));
      return true;
    } else {
      RCLCPP_WARN(
          logger_, "Failed to trigger transition %u", static_cast<unsigned int>(transition));
      return false;
    }
  }

 protected:
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
  rclcpp::Logger logger_{rclcpp::get_logger("lifecycle_client")};
  std::string node_to_control_;
};

void caller_script(LifecycleClient &client_one, LifecycleClient &client_two)
{
  rclcpp::WallRate time_between_state_changes(0.05);  // 10s

  // configure
  if (!client_one.change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
    return;
  }
  if (!client_one.get_state()) {
    return;
  }

  if (!client_two.change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
    return;
  }
  if (!client_two.get_state()) {
    return;
  }

  time_between_state_changes.sleep();

  //activate
  if (!client_one.change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
    return;
  }
  if (!client_one.get_state()) {
    return;
  }

  if (!client_two.change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
    return;
  }
  if (!client_two.get_state()) {
    return;
  }

  time_between_state_changes.sleep();

  //deactivate
  if (!client_one.change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
    return;
  }
  if (!client_one.get_state()) {
    return;
  }

  if (!client_two.change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
    return;
  }
  if (!client_two.get_state()) {
    return;
  }

  time_between_state_changes.sleep();

  //cleanup
  if (!client_one.change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
    return;
  }
  if (!client_one.get_state()) {
    return;
  }

  if (!client_two.change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
    return;
  }
  if (!client_two.get_state()) {
    return;
  }

//  time_between_state_changes.sleep();
}

}  // namespace the_node

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<rclcpp::Node>("manager");
  the_node::LifecycleClient client_one(node, "managed_node_one");
  the_node::LifecycleClient client_two(node, "managed_node_two");

  std::unique_ptr<nav2_util::NodeThread> node_thread = std::make_unique<nav2_util::NodeThread>(node);
  the_node::caller_script(client_one, client_two);

  rclcpp::shutdown();
  return 0;
}