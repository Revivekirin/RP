#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("topic_exam_publisher");

  auto pub = node->create_publisher<std_msgs::msg::String>("topic_exam_message", 10);
  rclcpp::WallRate loop_rate(10.0);  // 10 Hz

  int count = 0;
  while (rclcpp::ok()) {
    std_msgs::msg::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();  // ← 세미콜론 필수

    RCLCPP_INFO(node->get_logger(), "%s", msg.data.c_str());

    pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
    ++count;
  }

  rclcpp::shutdown();
  return 0;
}
