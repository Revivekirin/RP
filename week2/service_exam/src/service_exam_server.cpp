#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "service_exam/srv/my_service.hpp"

using MyService = service_exam::srv::MyService;

class ServiceExamServer : public rclcpp::Node {
public:
  ServiceExamServer() : rclcpp::Node("service_exam_server") {
    service_ = this->create_service<MyService>(
      "my_service",
      std::bind(&ServiceExamServer::handle_service, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    RCLCPP_INFO(this->get_logger(), "my_service: ready");
  }

private:
  void handle_service(const std::shared_ptr<rmw_request_id_t>,
                      const std::shared_ptr<MyService::Request> req,
                      std::shared_ptr<MyService::Response> res) {
    res->result = req->a + req->b;
    RCLCPP_INFO(this->get_logger(), "request: a=%ld, b=%ld",
                static_cast<long>(req->a), static_cast<long>(req->b));
    RCLCPP_INFO(this->get_logger(), "sending back response [%ld]",
                static_cast<long>(res->result));
  }

  rclcpp::Service<MyService>::SharedPtr service_;  
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceExamServer>());
  rclcpp::shutdown();
  return 0;
}
