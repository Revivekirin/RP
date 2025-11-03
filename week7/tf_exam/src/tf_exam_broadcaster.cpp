#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tf_exam_broadcaster");
  rclcpp::Rate rate(100);

  // Transform Broadcaster 생성
  auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  while (rclcpp::ok())
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = node->get_clock()->now();
    transformStamped.header.frame_id = "coordinate_vehicle";
    transformStamped.child_frame_id = "coordinate_sensor";

    // Translation
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = -0.1;
    transformStamped.transform.translation.z = 0.4;

    // Rotation (Quaternion)
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    // Publish transform
    broadcaster->sendTransform(transformStamped);
     

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
