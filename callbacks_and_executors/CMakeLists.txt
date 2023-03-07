cmake_minimum_required(VERSION 3.8)
project(callbacks_and_executors)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(${PROJECT_NAME}
  src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp)


install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
