#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <chrono>

using namespace std::chrono_literals;


class RecursiveCall : public rclcpp::Node
{
 public:
  RecursiveCall() : Node("recursive_call")
  {
    serviceA_ = create_service<std_srvs::srv::SetBool>(
        "service_A",
        std::bind(&RecursiveCall::handle_serviceA, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3)
        );

    serviceB_ = create_service<std_srvs::srv::SetBool>(
        "service_B",
        std::bind(&RecursiveCall::handle_serviceB, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3)
    );

    serviceA_client_ = create_client<std_srvs::srv::SetBool>("service_A");
  }

  void handle_serviceA(
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<std_srvs::srv::SetBool::Request>,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service A");
    response->success = true;
  }

  void handle_serviceB(
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<std_srvs::srv::SetBool::Request>,
      const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service B");

    while (!serviceA_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = serviceA_client_->async_send_request(request);


    response->success = true;
  }

 private:
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr serviceA_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr serviceA_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr serviceB_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = RecursiveCall();
  rclcpp::spin(node.get_node_base_interface());

  return 0;
}

