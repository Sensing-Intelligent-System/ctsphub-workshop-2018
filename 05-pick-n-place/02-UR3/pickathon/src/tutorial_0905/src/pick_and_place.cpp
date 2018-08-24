#include <ros/ros.h>
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
  
  // Reset apriltag_0 pose in gazebo.
  gazebo_msgs::ModelState model_state;
  model_state.model_name = "apriltag_0";
  model_state.pose = make_pose(0.25, 0.4, 0.8, 1.5708, 0, 0);
  gazebo_msgs::SetModelState srv_gz;
  srv_gz.request.model_state = model_state;
  gazebo_client.call(srv_gz);

  // Go to pose which surveys where the object is to be picked.
  tutorial_0905::SetTargetPose srv;
  // ************************* Student Implementation Part 4 *************************
  // Please use recommended pose here: 
  // (x, y, z) = [0.15, 0.25, 0.25]
  // (q_x, q_y, q_z, q_w) = [-0.653268, -0.270633, 0.653268, 0.270633]
  srv.request.target_pose = make_pose(0.15, 0.25, 0.25, -0.653268, -0.270633, 0.653268, 0.270633);
  srv.request.gripping_force = 0;
  srv.request.planning_time = 5.0;
  ur3_client.call(srv);
  // ************************* Student Implementation Part 4 *************************
  ROS_INFO_STREAM(srv.response.plan_result);

  // Pick object in following waypoints.
  double depth[2] = {-0.02, 0.08}, force = argc == 2 ? atof(argv[1]) : 50;
  geometry_msgs::Pose waypoint[2];
  pose_from_apriltag("obj_to_pick", waypoint, depth);
  srv.request.target_pose = waypoint[0];
  srv.request.gripping_force = 0;
  srv.request.planning_time = 3.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result); 

  srv.request.target_pose = waypoint[1];
  srv.request.gripping_force = force;
  srv.request.planning_time = 2.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result); 

  // Go to pose which surveys where the object is to be placed.
  // ************************* Student Implementation Part 5 *************************
  // Please use recommended pose here: 
  // (x, y, z) = [-0.15, 0.25, 0.25]
  // (q_x, q_y, q_z, q_w) = [-0.653268, -0.270633, 0.653268, 0.270633]
  srv.request.target_pose = make_pose(-0.15, 0.25, 0.25, -0.653268, -0.270633, 0.653268, 0.270633);
  srv.request.gripping_force = force;
  srv.request.planning_time = 5.0;
  ur3_client.call(srv);
  // ************************* Student Implementation Part 5 *************************
  ROS_INFO_STREAM(srv.response.plan_result);

  // Place object in following waypoints.
  pose_from_apriltag("where_to_place", waypoint);
  srv.request.target_pose = waypoint[0];
  srv.request.gripping_force = force;
  srv.request.planning_time = 3.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result); 

  srv.request.target_pose = waypoint[1];
  srv.request.gripping_force = 0;
  srv.request.planning_time = 2.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result); 

  return 0;
}
