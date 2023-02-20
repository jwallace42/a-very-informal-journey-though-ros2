#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace the_node
{


class ManagedNodeTwo : public rclcpp_lifecycle::LifecycleNode
{
 public:
  explicit ManagedNodeTwo(const rclcpp::NodeOptions &options) :
      rclcpp_lifecycle::LifecycleNode("node_two", options)
  {}
};

} // namespace the_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(the_node::ManagedNodeTwo)