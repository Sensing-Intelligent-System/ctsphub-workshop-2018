#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

geometry_msgs::Pose make_pose(double x, double y, double z,
                              double q_x, double q_y, double q_z, double q_w) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = q_x;
  pose.orientation.y = q_y;
  pose.orientation.z = q_z;
  pose.orientation.w = q_w;
  
  return pose;
}

geometry_msgs::Pose make_pose(double x, double y, double z,
                              double roll, double pitch, double yaw) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  
  return pose;
}

int pose_from_apriltag(std::string str, geometry_msgs::Pose* waypoint, double* depth=NULL) {
  static tf::TransformBroadcaster br;
  tf::StampedTransform tf_obj;
  tf::Transform tf_final, tf_fetch;
  tf::TransformListener listener;
  
  // Look up transformation of aprilatag relative to ur3's coordination
  try {
    listener.waitForTransform("/base_link", str,
                             ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("/base_link", str,   
                             ros::Time(0), tf_obj);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return -1;
  }

  // Transform apriltag coordinate to align tool-center-point
  tf_fetch.setOrigin(tf::Vector3(0, 0, 0));
  tf_fetch.setRotation(tf::createQuaternionFromRPY(M_PI/2, 1.5708, 1.5708));
  
  tf_final = tf_obj*tf_fetch;
  tf::Vector3 col = tf_final.getBasis().getColumn(0);
  
  for (int i = 0; i < 2; ++ i) {
    double dist = depth == NULL ? -0.03 : depth[i];
    tf_final.setOrigin(
      tf::Vector3(tf_final.getOrigin().getX() + dist*col.getX(),
                  tf_final.getOrigin().getY() + dist*col.getY(),
                  tf_final.getOrigin().getZ() + dist*col.getZ()));

    waypoint[i] = make_pose(tf_final.getOrigin().getX(), 
                            tf_final.getOrigin().getY(),
                            tf_final.getOrigin().getZ(),
                            tf_final.getRotation().getX(),
                            tf_final.getRotation().getY(),
                            tf_final.getRotation().getZ(),
                            tf_final.getRotation().getW());
  }
  return 0;
}
