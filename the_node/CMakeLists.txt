cmake_minimum_required(VERSION 3.8)
project(the_node)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

set(dependencies 
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
)

add_library(managed_node_one SHARED
  src/managed_node_one.cpp
)

ament_target_dependencies(managed_node_one
  ${dependencies}
)

add_library(managed_node_two SHARED
    src/managed_node_two.cpp
)

ament_target_dependencies(managed_node_two
    ${dependencies}
)

add_library(managed_node_starter SHARED
    src/managed_node_starter.cpp
)

ament_target_dependencies(managed_node_starter
    ${dependencies}
)


rclcpp_components_register_nodes(managed_node_one "the_node::ManagedNodeOne")
rclcpp_components_register_nodes(managed_node_two "the_node::ManagedNodeTwo")
rclcpp_components_register_nodes(managed_node_starter "the_node::ManagedNodeStarter")


install(TARGETS managed_node_one managed_node_two managed_node_starter
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
