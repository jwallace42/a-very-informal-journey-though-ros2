#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("node");

  auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_callback_group(callback_group, node->get_node_base_interface());

  auto callback = []()
  {
    std::cout << "Running timer callback" << std::endl;
  };

  auto timer = node->create_wall_timer(1s, callback);

  auto callback_groups = executor.get_all_callback_groups();

  std::cout << "Callback group size" << callback_groups.size() << std::endl;

  executor.add_node(node);

  executor.spin();
  return 0;
}