#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

namespace the_node {

class ManagedNodeStarter : public rclcpp::Node {
 public:
  explicit ManagedNodeStarter(const rclcpp::NodeOptions &options)
      : rclcpp::Node("starter", options)
  {}
};

}  // namespace the_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(the_node::ManagedNodeStarter)