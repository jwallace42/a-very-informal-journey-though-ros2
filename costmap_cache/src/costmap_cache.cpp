#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "rosbag2_cpp/writer.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hello");

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  writer_ = std::make_unique<rosbag2_cpp::Writer>();

  
  writer_->open("test_bag");
  nav_msgs::msg::OccupancyGrid occ_grid_msg;
  occ_grid_msg.header.frame_id = "map";
  occ_grid_msg.info.resolution = 0.1;
  occ_grid_msg.info.width = 10;
  occ_grid_msg.info.height = 10;
  occ_grid_msg.info.origin.orientation.w = 1.0;
  occ_grid_msg.data.assign(10 * 10, 100);

  rclcpp::Time time_stamp = node->now();
  std::string topic_name = "occupancy_grid";
  writer_->write<nav_msgs::msg::OccupancyGrid>(occ_grid_msg, topic_name, time_stamp);

  occ_grid_msg.data.assign(10 * 10, 0);
  rclcpp::Duration one_second(1, 0);
  time_stamp = node->now() + one_second;
  writer_->write<nav_msgs::msg::OccupancyGrid>(occ_grid_msg, topic_name, time_stamp);

  return 0;
}
