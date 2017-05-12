#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <math.h>

#define PI 3.14159265

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  moveit::planning_interface::MoveGroupInterface group("arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.75;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.6;
  group.setPoseTarget(target_pose1);
  group.move();

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);

  geometry_msgs::Pose target_pose2 = target_pose1;

  for (float theta = 0; theta < 10 * PI; theta += 0.05 * PI)  {
    target_pose2.position.y = 0.0 + 0.2 * sin (theta);
    target_pose2.position.z = 0.7 + 0.2 * cos (theta);
    waypoints.push_back(target_pose2);
  }

  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints, 0.05, 0.0, trajectory, false);

  ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)", fraction * 100.0);

  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm");
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool success = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

  rt.getRobotTrajectoryMsg(trajectory);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  group.execute(plan);
}
