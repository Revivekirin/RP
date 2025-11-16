#include <rclcpp/rclcpp.hpp>

// MoveIt 2
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <iostream>

// Location class definition for representing 2D coordinates 
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
  std::string shape; // "box", "cylinder", "triangle"
  Block() = default;
  Block(double width_value, double length_value, double height_value, double radius_value, 
        Location location_value, std::string shape_type = "box")
    : width(width_value), length(length_value), height(height_value), 
      radius(radius_value), location(location_value), shape(shape_type) {}
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
                     std::vector<geometry_msgs::msg::Pose> waypoints,
                     double max_velocity_scaling = 1.0)
{
  auto logger = node->get_logger();
  RCLCPP_INFO(logger, "way_point");

  // Set velocity scaling for slower movement
  move_group_interface.setMaxVelocityScalingFactor(max_velocity_scaling);

  moveit_msgs::msg::RobotTrajectory trajectory;

  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
  RCLCPP_INFO(logger, "Fraction score : fraction=%.3f", fraction);
  
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;
  
  move_group_interface.execute(cartesian_plan);
  
  // Reset to normal speed
  move_group_interface.setMaxVelocityScalingFactor(1.0);
}

class PickAndPlaceNode : public rclcpp::Node {
public:
  PickAndPlaceNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()) 
  : Node("pick_and_place", node_options) {
    RCLCPP_INFO(this->get_logger(), "Pick-and-Place Node started.");
  }

  void run() {
    auto node_ptr = this->shared_from_this();
    moveit::planning_interface::MoveGroupInterface arm(node_ptr, "manipulator");
    moveit::planning_interface::MoveGroupInterface gripper(node_ptr, "gripper");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    arm.setPlanningTime(10.0);
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Ground plane collision object
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
    open_gripper(gripper);
    
    // Extra stabilization time after initialization
    RCLCPP_INFO(this->get_logger(), "Initial stabilization...");
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Define all 8 objects with their initial and target locations
    // Reordered: Boxes (1-6), Cylinder (7), Triangle (8)
    // Order changed: box1, box2, box3, box4, box5, box6, cylinder, triangle
    Block objects[] = {
      Block(0.05, 0.05, 0.06, 0.03, Location(0.4, 0.1), "box"),       // 1: box1 (height: 0.06, z_offset: 0.03)
      Block(0.05, 0.05, 0.06, 0.03, Location(0.4, 0.0), "box"),       // 2: box2 (height: 0.06, z_offset: 0.03)
      Block(0.05, 0.025, 0.06, 0.03, Location(0.4, -0.1), "box"),     // 3: box3 (height: 0.06, z_offset: 0.03, needs rotation)
      Block(0.05, 0.05, 0.07, 0.035, Location(0.5, -0.1), "box"),     // 4: box4 (height: 0.07, z_offset: 0.035)
      Block(0.05, 0.05, 0.08, 0.04, Location(0.5, 0.1), "box"),       // 5: box5 (height: 0.08, z_offset: 0.04) 
      Block(0.05, 0.05, 0.09, 0.045, Location(0.6, 0.0), "box"),      // 6: box6 (height: 0.09, z_offset: 0.045) 
      Block(0.05, 0.05, 0.06, 0.03, Location(0.5, 0.0), "cylinder"),  // 7: cylinder (height: 0.06, z_offset: 0.03, slow approach)
      Block(0.05, 0.05, 0.06, 0.03, Location(0.6, 0.1), "triangle")   // 8: triangle (height: 0.06, z_offset: 0.03, gentle grip)
    };

    // Target locations
    // Swapped: box5 and box6 order changed, box6 and cylinder targets swapped
    Location target_locations[] = {
      Location(-0.15, 0.45),  // 1: box1 target
      Location(-0.15, 0.55),  // 2: box2 target
      Location(0.15, 0.45),   // 3: box3 target
      Location(0.15, 0.55),   // 4: box4 target
      Location(0.05, 0.55),  // 5: box5 target 
      Location(-0.05, 0.55),  // 6: box6 target 
      Location(-0.05, 0.45),   // 7: cylinder target 
      Location(0.05, 0.45)    // 8: triangle target
    };

    // Gripper values - adjusted triangle grip to be gentler
    double gripper_values[] = {
      0.0243,  // box1
      0.020,   // box2
      0.0100,  // box3
      0.020,   // box4
      0.020,   // box6
      0.020,   // box5
      0.0180,  // cylinder
      0.0130   // triangle (gentler grip: 0.010 -> 0.0150)
    };

    // Pick and place all objects in new order
    for (int i = 0; i < 8; i++) {
      RCLCPP_INFO(this->get_logger(), "=== Processing Object %d (%s) ===", i + 1, objects[i].shape.c_str());
      
      // Special handling for cylinder (index 6) - slow approach
      bool use_slow_approach = (i == 6);
      
      // Special handling for triangle (index 7) - gentle grip and slow approach
      bool use_gentle_approach = (i == 7);
      if (use_gentle_approach) {
        use_slow_approach = true; // Triangle also needs slow approach
      }
      
      // Special handling for box3 (index 2) - needs 90 degree rotation
      bool rotate_90 = (i == 2);
      
      RCLCPP_INFO(this->get_logger(), "Gripper value for this object: %.4f", gripper_values[i]);
      
      // Pick object from initial location
      pick_object(node_ptr, arm, gripper, objects[i], gripper_values[i], use_slow_approach);
      
      // Place object at target location
      place_object(node_ptr, arm, gripper, target_locations[i], objects[i].height, rotate_90);
      
      // Return to initial pose between objects
      initial_pose(arm);
      
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    RCLCPP_INFO(this->get_logger(), "All objects placed successfully!");
  }

private:
  void pick_object(rclcpp::Node::SharedPtr node,
                   moveit::planning_interface::MoveGroupInterface& arm_interface,
                   moveit::planning_interface::MoveGroupInterface& gripper_interface,
                   Block block, double gripper_value, bool use_slow_approach = false) {

    double grasp_height = block.height + 0.185;
    
    double yaw_angle = -M_PI/4;  // Default for all objects
    if (block.shape == "triangle") {
      yaw_angle = -M_PI/4 + M_PI/6;  // -45+ 30 = -15 (to grip edge/vertex)
      RCLCPP_INFO(node->get_logger(), "Triangle: adjusting yaw to grip edge (yaw: %.3f rad = %.1f deg)", 
                  yaw_angle, yaw_angle * 180.0 / M_PI);
    }
    
    RCLCPP_INFO(node->get_logger(), "Object: %s, Height: %.3f, Grasp height: %.3f", 
                block.shape.c_str(), block.height, grasp_height);
    
    // Approach position (above object, safe height)
    geometry_msgs::msg::Pose approach_pose = list_to_pose(
      block.location.x, block.location.y, 0.35, M_PI, 0, yaw_angle
    );
    
    // Grasp position (using original working formula)
    geometry_msgs::msg::Pose grasp_pose = list_to_pose(
      block.location.x, block.location.y, grasp_height, M_PI, 0, yaw_angle
    );
    
    // Lift position (above object with object in gripper)
    geometry_msgs::msg::Pose lift_pose = list_to_pose(
      block.location.x, block.location.y, 0.35, M_PI, 0, yaw_angle
    );

    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Move to approach position
    waypoints.clear();
    waypoints.push_back(approach_pose);
    waypoint_sample(arm_interface, node, waypoints);
    
    // Open gripper
    open_gripper(gripper_interface);
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    // Move down to grasp position
    // Use slower speed if requested (for cylinder and triangle)
    waypoints.clear();
    waypoints.push_back(grasp_pose);
    if (use_slow_approach) {
      RCLCPP_INFO(node->get_logger(), "Using slow approach for this object");
      waypoint_sample(arm_interface, node, waypoints, 0.3); // 30% speed
    } else {
      waypoint_sample(arm_interface, node, waypoints);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    // Close gripper
    close_gripper(gripper_interface, gripper_value);
    
    // Extra wait time for triangle to stabilize grip
    if (block.shape == "triangle") {
      rclcpp::sleep_for(std::chrono::milliseconds(800));
      RCLCPP_INFO(node->get_logger(), "Triangle: extra stabilization time");
    } else {
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    // Lift object (slower for triangle)
    waypoints.clear();
    waypoints.push_back(lift_pose);
    if (block.shape == "triangle") {
      waypoint_sample(arm_interface, node, waypoints, 0.3); // Lift triangle slowly
      RCLCPP_INFO(node->get_logger(), "Triangle: slow lift");
    } else {
      waypoint_sample(arm_interface, node, waypoints);
    }
  }

  void place_object(rclcpp::Node::SharedPtr node,
                    moveit::planning_interface::MoveGroupInterface& arm_interface,
                    moveit::planning_interface::MoveGroupInterface& gripper_interface,
                    Location target_location, double object_height, bool rotate_90 = false) {

    // Calculate yaw angle based on rotation requirement
    double yaw_angle = rotate_90 ? (-M_PI/4 + M_PI/2) : -M_PI/4;

    // Approach position above target
    geometry_msgs::msg::Pose approach_pose = list_to_pose(
      target_location.x, target_location.y, 0.35, M_PI, 0, yaw_angle
    );
    
    // Place position at target
    geometry_msgs::msg::Pose place_pose = list_to_pose(
      target_location.x, target_location.y, 0.125 + 0.185, M_PI, 0, yaw_angle
    );

    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Move to approach position above target
    waypoints.clear();
    waypoints.push_back(approach_pose);
    waypoint_sample(arm_interface, node, waypoints);

    // Move down to place position
    waypoints.clear();
    waypoints.push_back(place_pose);
    waypoint_sample(arm_interface, node, waypoints);
    rclcpp::sleep_for(std::chrono::milliseconds(300));

    // Open gripper to release object
    open_gripper(gripper_interface);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Move back up
    waypoints.clear();
    waypoints.push_back(approach_pose);
    waypoint_sample(arm_interface, node, waypoints);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndPlaceNode>();

  std::thread spinner([node]() {
    rclcpp::spin(node);
  });

  spinner.detach(); 

  node->run();
  rclcpp::shutdown();
  return 0;
}