#include <rclcpp/rclcpp.hpp>

// MoveIt 2
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <iostream>

// Location class definition for representing 2D coordinates 
// usually used for block locations or target locations
class Location {
public:
  double x;
  double y;
  Location() = default;
  Location(double x_value, double y_value) : x(x_value), y(y_value) {}
};

// Block class definition for representing blocks entities
class Block {
public:
  double width;
  double length;
  double height;
  double radius;
  Location location;
  Block() = default;
  Block(double width_value, double length_value, double height_value, double radius_value, Location location_value)
    : width(width_value), length(length_value), height(height_value), radius(radius_value), location(location_value) {}
};

// Function to convert position and orientation values to a Pose message
geometry_msgs::msg::Pose list_to_pose(double x, double y, double z, double roll, double pitch, double yaw) {
  geometry_msgs::msg::Pose pose;
  tf2::Quaternion orientation;
  orientation.setRPY(roll, pitch, yaw);
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = tf2::toMsg(orientation);
  return pose;
}

// Function to move the robot arm to a specified pose goal
void go_to_pose_goal(moveit::planning_interface::MoveGroupInterface& move_group_interface,
                     geometry_msgs::msg::Pose& target_pose) {
  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  auto planning_result = move_group_interface.plan(my_plan);
  bool success = (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    move_group_interface.execute(my_plan);
  }
}

// Function to control the gripper to close
void close_gripper(moveit::planning_interface::MoveGroupInterface& gripper_interface, double value) {
  RCLCPP_INFO(rclcpp::get_logger("gripper"), "Gripper closing...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions = gripper_interface.getCurrentJointValues();
  joint_group_positions[0] = value;
  joint_group_positions[1] = 0;
  gripper_interface.setJointValueTarget(joint_group_positions);
  auto planning_result = gripper_interface.plan(my_plan);
  bool success = (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    gripper_interface.execute(my_plan);
  }
}

// Function to control the gripper to open
void open_gripper(moveit::planning_interface::MoveGroupInterface& gripper_interface) {
  RCLCPP_INFO(rclcpp::get_logger("gripper"), "Gripper opening...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions = gripper_interface.getCurrentJointValues();
  joint_group_positions[0] = 0.07;
  joint_group_positions[1] = 0;
  gripper_interface.setJointValueTarget(joint_group_positions);
  auto planning_result = gripper_interface.plan(my_plan);
  bool success = (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    gripper_interface.execute(my_plan);
  }
}

void initial_pose(moveit::planning_interface::MoveGroupInterface& arm_interface) {
  RCLCPP_INFO(rclcpp::get_logger("arm"), "Moving to initial pose...");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions = arm_interface.getCurrentJointValues();
  joint_group_positions = {0, -2.03, 1.58, -1.19, -1.58, 0.78};
  arm_interface.setJointValueTarget(joint_group_positions);
  auto planning_result = arm_interface.plan(my_plan);
  bool success = (planning_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
      arm_interface.execute(my_plan);
  }
}

void waypoint_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                     rclcpp::Node::SharedPtr node,
                     std::vector<geometry_msgs::msg::Pose> waypoints)
{
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "way_point");

  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
  RCLCPP_INFO(logger, "Fraction score : fraction=%.3f", fraction);
  
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;
  
  move_group_interface.execute(cartesian_plan);
}

class PickAndLiftNode : public rclcpp::Node {
public:
  PickAndLiftNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()) 
  : Node("pick_and_lift", node_options) {
    RCLCPP_INFO(this->get_logger(), "Pick-and-Lift Node started.");
  }

  void run() {
    auto node_ptr = this->shared_from_this();
    moveit::planning_interface::MoveGroupInterface arm(node_ptr, "manipulator");
    moveit::planning_interface::MoveGroupInterface gripper(node_ptr, "gripper");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    arm.setPlanningTime(10.0);
    rclcpp::sleep_for(std::chrono::seconds(1));

    // ground plane
    moveit_msgs::msg::CollisionObject ground_plane;
    ground_plane.header.frame_id = arm.getPlanningFrame();
    ground_plane.id = "ground_plane";

    shape_msgs::msg::SolidPrimitive plane_primitive;
    plane_primitive.type = plane_primitive.BOX;
    plane_primitive.dimensions = {4.0, 4.0, 0.01};

    geometry_msgs::msg::Pose plane_pose;
    plane_pose.orientation.w = 1.0;
    plane_pose.position.z = -0.005;

    ground_plane.primitives.push_back(plane_primitive);
    ground_plane.primitive_poses.push_back(plane_pose);
    ground_plane.operation = ground_plane.ADD;
    planning_scene_interface.applyCollisionObjects({ground_plane});

    initial_pose(arm);
    close_gripper(gripper, 0.0);

    Block block[] = {
      Block(0.05, 0.05, 0.06, 0, Location(0.4, 0.1)), // Block 1
      Block(0.05, 0.05, 0.06, 0, Location(0.4, -0.1)), // Block 2
    };

    lift_block(node_ptr, arm, gripper, block[0], 0.0243);
    place_block(node_ptr, arm, gripper, Location(block[1].location.x, block[1].location.y));
  }

private:
  void lift_block(rclcpp::Node::SharedPtr node,
                  moveit::planning_interface::MoveGroupInterface& arm_interface,
                  moveit::planning_interface::MoveGroupInterface& gripper_interface,
                  Block block, double value) {

    geometry_msgs::msg::Pose target_pose1 = list_to_pose(block.location.x, block.location.y, 0.4, M_PI, 0, -M_PI/4);
    geometry_msgs::msg::Pose target_pose2 = list_to_pose(block.location.x, block.location.y, block.height + 0.185, M_PI, 0, -M_PI/4);
    
    std::vector<geometry_msgs::msg::Pose> waypoints1;
    std::vector<geometry_msgs::msg::Pose> waypoints2;
    std::vector<geometry_msgs::msg::Pose> waypoints3;

    waypoints1.push_back(target_pose1);
    waypoint_sample(arm_interface, node, waypoints1);
    open_gripper(gripper_interface);

    waypoints2.push_back(target_pose2);
    waypoint_sample(arm_interface, node, waypoints2);
    close_gripper(gripper_interface, value);

    waypoints3.push_back(target_pose1);
    waypoint_sample(arm_interface, node, waypoints3);

  }
  void place_block(rclcpp::Node::SharedPtr node,
                   moveit::planning_interface::MoveGroupInterface& arm_interface,
                   moveit::planning_interface::MoveGroupInterface& gripper_interface,
                   Location location) {

    geometry_msgs::msg::Pose target_pose1 = list_to_pose(location.x, location.y, 0.4, M_PI, 0, -M_PI/4);
    geometry_msgs::msg::Pose target_pose2 = list_to_pose(location.x, location.y, 0.125 + 0.185, M_PI, 0, -M_PI/4);
    
    std::vector<geometry_msgs::msg::Pose> waypoints1;
    std::vector<geometry_msgs::msg::Pose> waypoints2;
    
    waypoints1.push_back(target_pose1);
    waypoints1.push_back(target_pose2);
    waypoint_sample(arm_interface, node, waypoints1);
    open_gripper(gripper_interface);

    waypoints2.push_back(target_pose1);
    waypoint_sample(arm_interface, node, waypoints2);

  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndLiftNode>();

  std::thread spinner([node]() {
    rclcpp::spin(node);
  });

  spinner.detach(); 

  node->run();
  rclcpp::shutdown();
  return 0;
}
