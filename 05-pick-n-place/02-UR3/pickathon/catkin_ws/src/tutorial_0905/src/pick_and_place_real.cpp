#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tutorial_0905/SetTargetPose.h>
#include <tutorial_0905/MyPose.h>

#include "widget.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place");
  ros::NodeHandle nh;
  ros::ServiceClient ur3_client = nh.serviceClient<tutorial_0905::SetTargetPose>("/ur3_control/goto_pose");
  ros::ServiceClient ur3_home_srv = nh.serviceClient<std_srvs::Empty>("/ur3_control/goto_home");
 
  // Go to pose which surveys where the object is to be picked.
  tutorial_0905::SetTargetPose srv;
  // ************************* Student Implementation Part 4 *************************
  // Please use recommended pose here: 
  // (x, y, z) = [0.15, 0.25, 0.25]
  // (q_x, q_y, q_z, q_w) = [-0.653268, -0.270633, 0.653268, 0.270633]
  srv.request.target_pose = make_pose(0.2, 0.3, 0.35, -0.653268, -0.270633, 0.653268, 0.270633);
  srv.request.gripping_force = 0;
  srv.request.planning_time = 5.0;
  ur3_client.call(srv);
  // ************************* Student Implementation Part 4 *************************
  ROS_INFO_STREAM(srv.response.plan_result);

  // Pick object in following waypoints.
  double depth[2] = {-0.05, 0.05}, force = argc == 2 ? atof(argv[1]) : 50;
  geometry_msgs::Pose waypoint[2];
  while (ros::ok()) 
    if (!pose_from_apriltag("obj_to_pick", waypoint, depth)) break; 
  srv.request.target_pose = waypoint[0];
  srv.request.gripping_force = 0;
  srv.request.planning_time = 5.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result); 

  srv.request.target_pose = waypoint[1];
  srv.request.gripping_force = force;
  srv.request.planning_time = 3.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result); 

  // Go to pose which surveys where the object is to be placed.
  // ************************* Student Implementation Part 5 *************************
  // Please use recommended pose here: 
  // (x, y, z) = [-0.15, 0.25, 0.25]
  // (q_x, q_y, q_z, q_w) = [-0.653268, -0.270633, 0.653268, 0.270633]
  srv.request.target_pose = make_pose(-0.2, 0.3, 0.35, -0.653268, -0.270633, 0.653268, 0.270633);
  srv.request.gripping_force = force;
  srv.request.planning_time = 5.0;
  ur3_client.call(srv);
  // ************************* Student Implementation Part 5 *************************
  ROS_INFO_STREAM(srv.response.plan_result);

  // Place object in following waypoints.
  depth[0] = -0.1; depth[1] = -0.03;
  while (ros::ok())
    if (!pose_from_apriltag("where_to_place", waypoint, depth)) break;
  srv.request.target_pose = waypoint[0];
  srv.request.gripping_force = force;
  srv.request.planning_time = 5.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result); 

  srv.request.target_pose = waypoint[1];
  srv.request.gripping_force = 0;
  srv.request.planning_time = 3.0;
  ur3_client.call(srv);
  ROS_INFO_STREAM(srv.response.plan_result); 
  
  std_srvs::Empty home_srv;
  ur3_home_srv.call(home_srv);
  return 0;
}
