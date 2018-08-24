#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tutorial_0905/SetTargetPose.h>
#include <tutorial_0905/MyPose.h>

#include "widget.hpp"

int wait_key_play() {
  std::cout << "Waitkey: s to play the planning result, r to replan: " << std::endl;
  char c;
  std::cin >> c;
  if (c == 's') {
    return 1;  
  }
  else if (c == 'r')
  {
    return 2;
  } 
  else {
    std::cout << "End process." << std::endl;
    return 0;
  } 
}

int wait_key_execute() {
  std::cout << "Waitkey: s to execute the planning result" << std::endl;
  char c;
  std::cin >> c;
  if (c == 's') {
    return 1;  
  }else {
    std::cout << "End process." << std::endl;
    return 0;
  } 

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place");
  ros::NodeHandle nh;
  ros::ServiceClient ur3_client = nh.serviceClient<tutorial_0905::SetTargetPose>("/ur3_control/goto_pose");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // This connects to a running instance of the move_group node
  // Here the Planning group is "tcp"
  moveit::planning_interface::MoveGroupInterface group("tcp");
  group.setPlannerId("RRTConnectkConfigDefault");
  group.setPoseReferenceFrame("base_link");
  tutorial_0905::SetTargetPose srv;
  moveit::planning_interface::PlanningSceneInterface current_scene;

  // Go to pose which surveys where the object is to be picked.
  group.setPoseTarget(make_pose(0.2, 0.3, 0.35, -0.653268, -0.270633, 0.653268, 0.270633));
  // Motion plan from current location to custom position
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.plan(my_plan);
  while (1) {
    if (wait_key_play() == 1)
      break;
    else if (wait_key_play() == 2)
      group.plan(my_plan);
    else 
      return 0;
  }
  
  // Wait 3 second for trajectory visualization
  if (wait_key_execute()) {
    if (!group.move()) 
      ROS_ERROR("Fail to move to pose 1!");
  }
  else return 0;
  
  // Pick object in following waypoints.
  double depth[2] = {-0.02, 0.05}, force = argc == 2 ? atof(argv[1]) : 50;
  geometry_msgs::Pose waypoint[2];
  
  while (1) {
    pose_from_apriltag("obj_to_pick", waypoint, depth);
    group.setPoseTarget(waypoint[0]);
    group.plan(my_plan);
    int waitkey = wait_key_play();
    if (waitkey == 1)
      break;
    else if (waitkey == 0)
      return 0;
  }

  if(wait_key_execute()) {
    if (!group.move()) 
      ROS_ERROR("Fail to move to pose 2!");
  }
  else return 0;
  
  group.setPoseTarget(waypoint[1]);
  while (1) {
    group.plan(my_plan);
    int waitkey = wait_key_play();
    if (waitkey == 1)
      break;
    else if (waitkey == 0)
      return 0;
  }
  
  if(wait_key_execute()) {
    if (!group.move()) 
      ROS_ERROR("Fail to move to pose 3!");
  }
  else return 0;
  
  // Tighten the gripper
  srv.request.target_pose = waypoint[1];
  srv.request.gripping_force = force;
  srv.request.planning_time = 2.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result);

  // Go to pose which surveys where the object is to be placed.
  // Here we visualize the original plan first.
  group.setPoseTarget(make_pose(-0.2, 0.3, 0.35, -0.653268, -0.270633, 0.653268, 0.270633));
  while (1) {
    group.plan(my_plan);
    int waitkey = wait_key_play();
    if (waitkey == 1)
      break;
    else if (waitkey == 0)
      return 0;
  }

  // Let's add some obstacle into planning scene, aka the beer in scene
  moveit_msgs::CollisionObject box[2];
  box[0].id = "obstacle";
  box[1].id = "floor";
  
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.01;
  primitive.dimensions[1] = 0.3;
  primitive.dimensions[2] = 0.6;

  box[0].primitives.push_back(primitive);
  box[0].primitive_poses.push_back(make_pose(0, 0.45, 0.05, 0, 0, 0));
  box[0].operation = box[0].ADD;
  box[1].primitives.push_back(primitive);
  box[1].primitive_poses.push_back(make_pose(0, 0.45, 0.0, 0, 1.5708, 0));
  box[1].operation = box[1].ADD;
  
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(box[0]);
  collision_objects.push_back(box[1]);
  current_scene.applyCollisionObjects(collision_objects);
  //std::cin.get();

  // Go to pose which surveys where the object is to be placed.
  // Let's see if UR3 will generate a new plan being able to avoid obstacle.
  while (1) {
    group.plan(my_plan);
    int waitkey = wait_key_play();
    if (waitkey == 1)
      break;
    else if (waitkey == 0)
      goto END;
  }

  if(wait_key_execute()) {
    if (!group.move()) 
      ROS_ERROR("Fail to move to pose 4!");
  }
  else return 0;

  // Place object in following waypoints.
  depth[0] = -0.02; depth[1] = -0.03;
  while (1) {
    pose_from_apriltag("where_to_place", waypoint, depth);
    group.setPoseTarget(waypoint[0]);
    group.plan(my_plan);
    int waitkey = wait_key_play();
    if (waitkey == 1)
      break;
    else if (waitkey == 0)
      goto END;
  }
  if(wait_key_execute()) {
    if (!group.move()) 
      ROS_ERROR("Fail to move to pose 5!");
  }
  else return 0;

  group.setPoseTarget(waypoint[1]);
  while (1) {
    group.plan(my_plan);
    int waitkey = wait_key_play();
    if (waitkey == 1)
      break;
    else if (waitkey == 0)
      goto END;
  }
  if(wait_key_execute()) {
    if (!group.move()) 
      ROS_ERROR("Fail to move to pose 6!");
  }
  else return 0;
  // Relieve the gripper
  srv.request.target_pose = waypoint[1];
  srv.request.gripping_force = 0;
  srv.request.planning_time = 2.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result);

END:
  std::vector<std::string> obj;
  obj.push_back(box[0].id);
  obj.push_back(box[1].id);
  current_scene.removeCollisionObjects(obj);
  return 0;
}
