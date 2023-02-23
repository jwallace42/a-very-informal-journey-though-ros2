cmake_minimum_required(VERSION 3.8)
project(the_node)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

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

add_library(node_thread SHARED
    src/node_thread.cpp)

ament_target_dependencies(node_thread
    ${dependencies}
    )

add_executable(the_node_executable
    src/lifecycle_client_node.cpp
    )

ament_target_dependencies(the_node_executable
    ${dependencies}
)

target_link_libraries(the_node_executable node_thread)

rclcpp_components_register_nodes(managed_node_one "the_node::ManagedNodeOne")
rclcpp_components_register_nodes(managed_node_two "the_node::ManagedNodeTwo")

install(TARGETS managed_node_one managed_node_two the_node_executable node_thread
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS the_node_executable
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
