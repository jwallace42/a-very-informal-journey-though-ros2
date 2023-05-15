cmake_minimum_required(VERSION 3.8)
project(recursive_calls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

set(dependencies
    rclcpp
    std_srvs)

add_executable(recursive_call src/recursive_calls.cpp)
ament_target_dependencies(recursive_call ${dependencies})

install(TARGETS recursive_call
    RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
