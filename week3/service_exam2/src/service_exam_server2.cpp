#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "service_exam2/srv/my_service.hpp"  // 커스텀 srv

using namespace std::chrono_literals;

#define PLUS 1
#define MINUS 2
#define MULTIPLICATION 3
#define DIVISION 4

class ServiceExamServer : public rclcpp::Node
{
public:
  ServiceExamServer() : rclcpp::Node("service_exam_server2"), opt_(PLUS)
  {
    this->declare_parameter<int>("calculation_method", PLUS);

    service_ = this->create_service<service_exam2::srv::MyService>(
      "my_service",
      std::bind(&ServiceExamServer::handle_service, this,
                std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      100ms, std::bind(&ServiceExamServer::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "my_service: ready");
  }

private:
  void handle_service(
      const std::shared_ptr<service_exam2::srv::MyService::Request> request,
      std::shared_ptr<service_exam2::srv::MyService::Response> response)
  {
    int64_t a = request->a;
    int64_t b = request->b;
    int64_t out = 0;

    switch (opt_) {
      case PLUS:             out = a + b; break;
      case MINUS:            out = a - b; break;
      case MULTIPLICATION:   out = a * b; break;
      case DIVISION:         out = (b != 0 ? a / b : 0); break;  
      default:               out = 0; break;
    }

    response->result = out;
    RCLCPP_INFO(this->get_logger(), "Req: (%ld, %ld), opt=%d -> result=%ld",
                static_cast<long>(a), static_cast<long>(b), opt_, static_cast<long>(out));
  }

  void timer_callback()
  {
    (void)this->get_parameter("calculation_method", opt_);
  }

  rclcpp::Service<service_exam2::srv::MyService>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  int opt_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceExamServer>());
  rclcpp::shutdown();
  return 0;
}
