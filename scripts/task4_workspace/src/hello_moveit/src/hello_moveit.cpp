#include <cmath>

#include <pluginlib/class_loader.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/planning_scene.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_pipeline");

int node_num = 38;
int start_node = 0;
int end_node = 38;

double node_x[38] = {0.3,0.248,0.232,0.228,0.217,0.227,0.189,0.182,0.150,0.118,0.113,0.120,0.150,0.150,0.129,0.127,0.091,0.085,0.060,0.031,0.029,0.060,
  0.127,0.133,0.150,0.168,0.172,0.183,0.172,0.198,0.240,0.2825,0.240,0.250,0.272,0.290,0.272,0.3}; 

double node_y[38] = {0.0,0.013,0.0126,0.0257,0.0422,0.0529,0.0398,0.016,0.003,0.01,0.04,0.0685,0.078,0.112,0.112,0.127,0.140,0.117,0.109,0.116,0.140,0.171,
  0.153,0.168,0.165,0.166,0.152,0.140,0.127,0.140,0.181,0.140,0.098,0.066,0.053,0.0429,0.0263,0.0};

double node_yaw[38] = {0.0,90.0,60.0,30.0,0.0,-30.0,180.0,135.0,90.0,45.0,0.0,-45.0,-90.0,90.0,60.0,30.0,180.0,135.0,90.0,45.0,0.0,-90.0,-30.0,-45.0,-90.0,
  -90.0,-150.0,-120.0,150.0,0.0,-90.0,180.0,90.0,-90.0,-150.0,180.0,150.0,0.0};

int main(int argc, char * argv[])

{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("motion_planning_pipeline_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader(node, "robot_description"));

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(node, robot_model_loader));

  psm->startSceneMonitor();
  psm->startWorldGeometryMonitor();
  psm->startStateMonitor();

  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("arm");

  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node, "ompl"));

  move_group_interface.setEndEffectorLink("l4");
  move_group_interface.setPoseReferenceFrame("base_link");
  move_group_interface.setPlanningTime(10.0);
  move_group_interface.setNumPlanningAttempts(100);
  move_group_interface.setGoalOrientationTolerance(0.01); 
  move_group_interface.setGoalJointTolerance(0.01);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setStartStateToCurrentState();

  auto const current_state = move_group_interface.getCurrentState(10.0);
  if (!current_state)
  {
    RCLCPP_ERROR(LOGGER, "Failed to get the current robot state.");
    rclcpp::shutdown();
    return 1;
  }

  auto const current_pose = move_group_interface.getCurrentPose("l4");
  if (current_pose.header.frame_id.empty())
  {
    RCLCPP_ERROR(LOGGER, "Failed to get the current end effector pose.");
    rclcpp::shutdown();
    return 1;
  }

  auto target_pose = current_pose.pose;

  if(start_node + node_num > end_node)
  {
    node_num = end_node - start_node;
  }

  bool move_reported_success;
  bool success;
  std::vector<double> target_joint_values;
  tf2::Quaternion quat_tf2;
  geometry_msgs::msg::Quaternion quat_msg;
  moveit::core::RobotState target_state(*current_state);

  for(int i = start_node; i < start_node + node_num;i++)
  {
    target_pose.position.x = node_x[i];
    target_pose.position.y = node_y[i];

    quat_tf2.setRPY(3.14159, 0.0, node_yaw[i]/180.0*3.14159);
    quat_msg = tf2::toMsg(quat_tf2);

    target_pose.orientation.x = quat_msg.x;
    target_pose.orientation.y = quat_msg.y;
    target_pose.orientation.z = quat_msg.z;
    target_pose.orientation.w = quat_msg.w;

    if (!target_state.setFromIK(joint_model_group, target_pose, "l4", 1.0))
    {
      RCLCPP_ERROR(LOGGER, "Failed to solve IK for the requested l4 pose.");
      rclcpp::shutdown();
      return 1;
    }

    target_joint_values.clear();

    RCLCPP_ERROR(LOGGER, "count:%d",i);

    target_state.copyJointGroupPositions(joint_model_group, target_joint_values);
    move_group_interface.setJointValueTarget(target_joint_values);

    move_reported_success = static_cast<bool>(move_group_interface.move());
    success = move_reported_success;
  }

  rclcpp::shutdown();
  return 0;
}