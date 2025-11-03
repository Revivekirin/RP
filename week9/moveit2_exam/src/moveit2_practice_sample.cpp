#include <rclcpp/rclcpp.hpp>

// MoveIt2
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <iostream>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


void go_to_pose_goal(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                     geometry_msgs::msg::Pose &target_pose) 
{
  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success){
    move_group_interface.execute(my_plan);
  }
  
}

geometry_msgs::msg::Pose list_to_pose(double x,double y,double z,double roll,double pitch,double yaw)
{
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion orientation;
  orientation.setRPY(roll,pitch, yaw);
  target_pose.orientation = tf2::toMsg(orientation);
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  return target_pose;
}

void move_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                 rclcpp::Node::SharedPtr node)
{
  auto logger = node->get_logger();

  geometry_msgs::msg::Pose target_pose;
  target_pose = list_to_pose(0.4, 0.2, 0.5, -M_PI/2 ,0, 0);
  go_to_pose_goal(move_group_interface,target_pose);

  RCLCPP_INFO(logger, "Move along Y-axis");
  target_pose.position.y += 0.05;
  go_to_pose_goal(move_group_interface,target_pose);
  target_pose.position.y -= 0.1;
  go_to_pose_goal(move_group_interface,target_pose);
  target_pose.position.y += 0.05;
  go_to_pose_goal(move_group_interface,target_pose);

  RCLCPP_INFO(logger, "Move along X-axis");
  target_pose.position.x += 0.02;
  go_to_pose_goal(move_group_interface,target_pose);
  target_pose.position.x -= 0.05;
  go_to_pose_goal(move_group_interface,target_pose);
  target_pose.position.x += 0.02;
  go_to_pose_goal(move_group_interface,target_pose);

  RCLCPP_INFO(logger, "Move along Z-axis");
  target_pose.position.z += 0.1;
  go_to_pose_goal(move_group_interface,target_pose);
  target_pose.position.z -= 0.1;
  go_to_pose_goal(move_group_interface,target_pose);
}

void rotation_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  geometry_msgs::msg::Pose target_pose;
  target_pose = list_to_pose(0.34, 0.2, 0.3, -M_PI/2 ,0, 0);
  go_to_pose_goal(move_group_interface,target_pose);

  target_pose = list_to_pose(0.34, 0.2, 0.3, -M_PI/2 ,0.3, 0);
  go_to_pose_goal(move_group_interface,target_pose);

  target_pose = list_to_pose(0.34, 0.2, 0.3, -M_PI/2, -0.3, 0);
  go_to_pose_goal(move_group_interface,target_pose);

  target_pose = list_to_pose(0.34, 0.2, 0.3, -M_PI/2 ,0, 0);
  go_to_pose_goal(move_group_interface,target_pose);
}

void gripper_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                    double value, rclcpp::Node::SharedPtr node)
{
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "gripper sample"); 

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  std::vector<double> joint_group_positions;

  joint_group_positions = move_group_interface.getCurrentJointValues();
  joint_group_positions[0]=value;
  joint_group_positions[1]=0;

  move_group_interface.setJointValueTarget(joint_group_positions);
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success){
    move_group_interface.execute(my_plan);
  }
}

void waypoint_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                     rclcpp::Node::SharedPtr node)
{
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "way_point");

  geometry_msgs::msg::Pose target_pose;
  target_pose = list_to_pose(0.4, 0.2, 0.6, -M_PI/2 ,0, 0);
  go_to_pose_goal(move_group_interface,target_pose);

  rclcpp::sleep_for(std::chrono::seconds(2));

  geometry_msgs::msg::Pose start = target_pose;

  moveit_msgs::msg::RobotTrajectory trajectory;
  std::vector<geometry_msgs::msg::Pose> waypoints;

  start.position.z -= 0.1;
  waypoints.push_back(start);

  start.position.x += 0.1;
  waypoints.push_back(start);

  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
  RCLCPP_INFO(logger, "Fraction score : fraction=%.3f", fraction);
  
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;
  
  move_group_interface.execute(cartesian_plan);
}

void rectangle_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                      rclcpp::Node::SharedPtr node)
{
  
  // waypoint_sample 을 참고하여 구현하시오
  


}

void circle_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                   rclcpp::Node::SharedPtr node)
{

  // waypoint_sample 을 참고하여 구현하시오



}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true)}
  );
  
  auto node = rclcpp::Node::make_shared("moveit2_practice_sample",node_options);

  moveit::planning_interface::MoveGroupInterface arm(node, "manipulator");
  moveit::planning_interface::MoveGroupInterface gripper(node, "gripper");
  arm.setPlanningTime(45.0);

  node->declare_parameter<int>("cmd", 1);  
  int cmd;
  node->get_parameter("cmd", cmd);

  rclcpp::sleep_for(std::chrono::seconds(1));
  switch(cmd) {
    case 1:
      move_sample(arm, node);
      rclcpp::sleep_for(std::chrono::seconds(1));
      break;
    case 2:
      rotation_sample(arm);
      rclcpp::sleep_for(std::chrono::seconds(1));
      break;
    case 3:
      gripper_sample(gripper, 0.07, node);
      rclcpp::sleep_for(std::chrono::seconds(1));
      gripper_sample(gripper, 0.0, node);
      rclcpp::sleep_for(std::chrono::seconds(1));
      break;
    case 4:
      waypoint_sample(arm, node);
      rclcpp::sleep_for(std::chrono::seconds(1));
      break;
    case 5:
      rectangle_sample(arm, node);
      rclcpp::sleep_for(std::chrono::seconds(1));
      break;
    case 6:
      circle_sample(arm, node);
      rclcpp::sleep_for(std::chrono::seconds(1));
      break;
  }
  rclcpp::shutdown();
  return 0;
}
