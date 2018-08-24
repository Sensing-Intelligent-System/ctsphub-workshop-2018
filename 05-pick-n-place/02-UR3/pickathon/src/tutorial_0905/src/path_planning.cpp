#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <gazebo_msgs/SetModelState.h>
#include <tutorial_0905/SetTargetPose.h>
#include <tutorial_0905/MyPose.h>

#include "widget.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place");
  ros::NodeHandle nh;
  ros::ServiceClient ur3_client = nh.serviceClient<tutorial_0905::SetTargetPose>("/ur3_control/goto_pose");
  ros::ServiceClient gazebo_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Reset pose of apriltag_0, beer in gazebo.
  gazebo_msgs::ModelState model_state;
  model_state.model_name = "apriltag_0";
  model_state.pose = make_pose(0.25, 0.4, 0.8, 1.5708, 0, 0);
  gazebo_msgs::SetModelState srv_gz;
  srv_gz.request.model_state = model_state;
  gazebo_client.call(srv_gz);

  model_state.model_name = "beer";
  model_state.pose = make_pose(0, 1.0, 0.78, 0, 0, 0);
  srv_gz.request.model_state = model_state;
  gazebo_client.call(srv_gz);

  // This connects to a running instance of the move_group node
  // Here the Planning group is "tcp"
  moveit::planning_interface::MoveGroupInterface group("tcp");
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPoseReferenceFrame("base_link");
  tutorial_0905::SetTargetPose srv;
  moveit::planning_interface::PlanningSceneInterface current_scene;

  // Go to pose which surveys where the object is to be picked.
  group.setPoseTarget(make_pose(0.15, 0.25, 0.25, -0.653268, -0.270633, 0.653268, 0.270633));
  // Motion plan from current location to custom position
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.plan(my_plan);
  
  // Wait 3 second for trajectory visualization  
  ros::Duration(3).sleep();  
  if (!group.move()) 
    ROS_ERROR("Fail to move to pose 1!");

  // Pick object in following waypoints.
  double depth[2] = {-0.02, 0.04}, force = argc == 2 ? atof(argv[1]) : 10;
  geometry_msgs::Pose waypoint[2];
  pose_from_apriltag("obj_to_pick", waypoint, depth);
  group.setPoseTarget(waypoint[0]);
  group.plan(my_plan);
  ros::Duration(3).sleep();  
  if (!group.move()) 
    ROS_ERROR("Fail to move to pose 2!");

  group.setPoseTarget(waypoint[1]);
  group.plan(my_plan);
  ros::Duration(3).sleep();  
  if (!group.move()) 
    ROS_ERROR("Fail to move to pose 3!");
  // Tighten the gripper
  srv.request.target_pose = waypoint[1];
  srv.request.gripping_force = force;
  srv.request.planning_time = 2.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result);

  // Go to pose which surveys where the object is to be placed.
  // Here we visualize the original plan first.
  group.setPoseTarget(make_pose(-0.15, 0.25, 0.25, -0.653268, -0.270633, 0.653268, 0.270633));
  group.plan(my_plan);
  ros::Duration(3).sleep();

  // Let's add some obstacle into planning scene, aka the beer in Gazebo
  model_state.model_name = "beer";
  model_state.pose = make_pose(0, 0.3, 0.9, 0, 0, 0, 1);
  srv_gz.request.model_state = model_state;
  gazebo_client.call(srv_gz);

  moveit_msgs::CollisionObject cylinder;
  cylinder.id = "gazebo_beer";
  
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.23;
  primitive.dimensions[1] = 0.055;

  cylinder.primitives.push_back(primitive);
  cylinder.primitive_poses.push_back(make_pose(0, 0.3, 0.9 + 0.23/2, 0, 0, 0, 1));
  cylinder.operation = cylinder.ADD;
  
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(cylinder);
  current_scene.applyCollisionObjects(collision_objects);
  ros::Duration(3).sleep();

  // Go to pose which surveys where the object is to be placed.
  // Let's see if UR3 will generate a new plan being able to avoid obstacle.
  group.plan(my_plan);
  ros::Duration(3).sleep();
  if (!group.move()) 
    ROS_ERROR("Fail to move to pose 4!");

  // Place object in following waypoints.
  pose_from_apriltag("where_to_place", waypoint);
  group.setPoseTarget(waypoint[0]);
  group.plan(my_plan);
  ros::Duration(3).sleep();  
  if (!group.move()) 
    ROS_ERROR("Fail to move to pose 5!");

  group.setPoseTarget(waypoint[1]);
  group.plan(my_plan);
  ros::Duration(3).sleep();  
  if (!group.move()) 
    ROS_ERROR("Fail to move to pose 6!");
  // Relieve the gripper
  srv.request.target_pose = waypoint[1];
  srv.request.gripping_force = 0;
  srv.request.planning_time = 2.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result);

  std::vector<std::string> obj;
  obj.push_back(cylinder.id);
  current_scene.removeCollisionObjects(obj);
  return 0;
}
