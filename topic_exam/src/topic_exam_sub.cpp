#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("topic_exam_subscriber");

  auto sub = node->create_subscription<std_msgs::msg::String>(
    "topic_exam_message", 10,
    [node](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_INFO(node->get_logger(), "I heard: [%s]", msg->data.c_str());
    }
  );

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
