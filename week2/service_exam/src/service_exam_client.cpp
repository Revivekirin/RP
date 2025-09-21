#include "rclcpp/rclcpp.hpp"
#include "service_exam/srv/myservice.hpp"
#include <cstdlib>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;
using Myservice = service_exam::srv::myservice;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("service_exam_client");

    if (argc != 3) {
        RCLCPP_INFO(node->get_logger(), "usage: service_exam_client num1 num2");
        return 1;
    }
    auto client = node->create_client<Myservice>("myservice")

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting...");
    }

    auto request = std::make_shared<Myservice::Request>();
    request->a = std::atoll(argv[1]);
    request->b = std::atoll(argv[2]);

    auto future = client->async_send_request(request);

    auto ret = rclcpp::spin_unil_future_complete(node, future);
    if (ret == rclcpp::FutureReturnCode::SUCESS) {
        RCLCPP_INFO(node->get_logger(), "sum: %ld", static_cast<long>(future.get()->result));
    } else [
        RCLCPP_ERROR(node->get_logger(), "Failed to call service my_service");
        return 1;
    ]

    rclcpp::shutdown();
    return 0;
}