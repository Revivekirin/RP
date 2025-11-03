#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

float offset = 0.0;

class TfExamListener : public rclcpp::Node
{
public:
  TfExamListener()
  : Node("tf_exam_listener"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 1초마다 타이머 콜백 실행
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TfExamListener::tfp, this));
  }

private:
  void tfp()
  {
    geometry_msgs::msg::PointStamped laser_point;
    laser_point.header.frame_id = "coordinate_sensor";
    //laser_point.header.stamp = this->get_clock()->now();
    laser_point.header.stamp = rclcpp::Time(0);

    laser_point.point.x = 1.0;
    laser_point.point.y = 0.2 + offset;
    laser_point.point.z = 0.0;
    offset += 0.1;

    try
    {
      geometry_msgs::msg::PointStamped base_point;
      tf_buffer_.transform(laser_point, base_point, "coordinate_vehicle");

      RCLCPP_INFO(this->get_logger(),
        "sensor: (%.2f, %.2f, %.2f) -----> vehicle: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z,
        rclcpp::Time(base_point.header.stamp).seconds());
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TfExamListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
