#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
//#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
//#include <robotiq_c_model_control/CModel_robot_output.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <ur_kin/ur_kin.h>
#include <std_srvs/Empty.h>
#include <thread>

// Standby position
const double joint_standby[6] = {1.6598024368286133, -1.1725829283343714, 0.6662349700927734, -1.3835914770709437, -1.5942438284503382, -1.4681914488421839};
// Shelf position
const double joint_shelf[6] = {0.048524126410484314, -2.250532929097311, 1.784745216369629, -2.4896023909198206, -1.613234821950094, -1.5081117788897913};
// Gripper length
// 0.16 meter for robotiq
const double tcp_length = 0.22;
// Maxmimum joint speed
const double joint_speed = 3.0;
// Pick and place apriltag id (predefined)
const int pick_id = 2;
const int place_id = 5;

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm {
 private:
  TrajClient* traj_client_;
  control_msgs::FollowJointTrajectoryGoal goal_;  
  ros::NodeHandle nh_;
  ros::Subscriber sub_joint_state_;
  ros::Subscriber sub_tag_id_;
  ros::ServiceClient client_, octomap_client_;
  // tf
  tf::StampedTransform tf_tag2base_link_;
  tf::TransformListener listener_;
  tf::TransformBroadcaster br_;
  tf::Transform tf_tag_link2tag_, tf_tag_link2base_link_;
  std::thread *th_;
  bool exit_;

  double joint_[6]; // joint state
  int tag_id_; // detected apriltag id

  // Check if the joint is valid
  // Input: joint position
  // Output: valid joint position
  // if absolute value of angle greater than 2pi
  // then angle <- mod(angle, 2pi)
  // if angle < -pi
  // then angle <- angle + 2pi
  // else if angle > pi
  // then angle <- angle - 2pi
  
  double validAngle(double angle) {
    if (abs(angle) > 2*M_PI) angle = fmod(angle, 2*M_PI);
    if (angle < -M_PI) return 2*M_PI + angle;
    else if (angle > M_PI) return angle - 2*M_PI;
    else return angle;
  }

  // sub_joint_state_ callback function
  // change the value of joint_[6]
  void jointStateCallback(const sensor_msgs::JointState &msg) {
    for (int i = 0; i < 6; ++i)
      joint_[i] = msg.position[i];
  }

  // Added by Sean
  // sub_tag_id callback function
  // Publish tag_link, which the tcp should arrive
  void apriltagCallback(const apriltags_ros::AprilTagDetectionArray &msg) {
    // Make sure the array is not empty 
    if (msg.detections.size() == 0) return;
    for (int i=0; i < msg.detections.size(); ++i){
      if(msg.detections[i].id != pick_id && msg.detections[i].id != place_id) continue;
      std::string frameID;
      frameID = ( msg.detections[i].id==pick_id? "tag_2":"tag_5");
      try {
        // Wait for the transform
        listener_.waitForTransform("base_link", frameID, ros::Time(0), ros::Duration(5.0));
        listener_.lookupTransform("base_link", frameID, ros::Time(0), tf_tag2base_link_);   
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
      } 
      /*
      tf::Quaternion q(tf_tag2base_link_.getRotation().x(),
                       tf_tag2base_link_.getRotation().y(),
                       tf_tag2base_link_.getRotation().z(),
                       tf_tag2base_link_.getRotation().w()); 
      tf::Matrix3x3 m_(q);
      double roll_, pitch_, yaw_;
      m_.getRPY(roll_, pitch_, yaw_);
      */
      // Print for debuging
      //std::cout << "Roll: " << roll_ << "; pitch: " << pitch_ << "; yaw: " << yaw_ << std::endl;
      tf_tag_link2tag_.setOrigin(tf::Vector3(0, 0, 0));
      if(msg.detections[i].id == pick_id) {
        /*tf_tag_link2base_link_.setOrigin(tf::Vector3(tf_tag2base_link_.getOrigin().x()-0.03, // toward gripper positive z-axis
                                                     tf_tag2base_link_.getOrigin().y(),
                                                     tf_tag2base_link_.getOrigin().z())) ;
        tf_tag_link2base_link_.setRotation(tf::createQuaternionFromRPY(-3.1415, 1.5708, yaw_));*/
        tf_tag_link2tag_.setRotation(tf::createQuaternionFromRPY(3.1415, 1.5708, 0));
      }

      else 
        tf_tag_link2tag_.setRotation(tf::createQuaternionFromRPY(3.1415, 1.5708, 0.0));
      tf_tag_link2base_link_ = tf_tag2base_link_ * tf_tag_link2tag_;
      
      /*if(msg.detections[i].id == place_id) {
        tf::Vector3 col = tf_tag2base_link_.getBasis().getColumn(2); // Left base_link basis positive z-axis, prevent from avoiding to shelf
        tf_tag_link2base_link_.setOrigin(tf::Vector3(tf_tag_link2base_link_.getOrigin().getX() + 0.05 * col.getX() , 
                                                     tf_tag_link2base_link_.getOrigin().getY() + 0.05 * col.getY() ,
                                                     tf_tag_link2base_link_.getOrigin().getZ() + 0.05 * col.getZ()));
      }*/
      br_.sendTransform(tf::StampedTransform(tf_tag_link2base_link_, ros::Time::now(), "base_link", "tag_link"));    
    }
  }

  // Calculate the DH matrix from given pose
  // Input: pose(position& orientation), T(matrix that save the result)
  // Output: None
  void pose_to_DH(geometry_msgs::Pose pose, double *T) {
    geometry_msgs::Point &p = pose.position;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double sinr = sin(roll), cosr = cos(roll);
    double sinp = sin(pitch), cosp = cos(pitch);
    double siny = sin(yaw), cosy = cos(yaw);

    // DH matrix, ZYX convention
    // https://en.wikipedia.org/wiki/Euler_angles
    // Tait-Bryan angle, ZYX
    // Map tool0 back to ee_link
    T[0] = cosy*cosp;
    T[1] = cosy*sinp*sinr - cosr*siny;
    T[2] = siny*sinr + cosy*cosr*sinp;
    T[3] = p.x;
    T[4] = cosp*siny;
    T[5] = cosy*cosr + siny*sinp*sinr;
    T[6] = cosr*siny*sinp - cosy*sinr;
    T[7] = p.y;
    T[8] = -sinp;
    T[9] = cosp*sinr;
    T[10] = cosp*cosr;
    T[11] = p.z;
    T[15] = 1;
  }
  // Perform IK calculation
  // Input: target_pose(pose that ee_link going to be), sol(joint position container)
  // Output: None
  void perform_IK(geometry_msgs::Pose target_pose, double *sol) {
    double T[16] = {0};
    // TCP's DH Matrix
    pose_to_DH(target_pose, T);
    // ee_link's DH Matrix
    // tcp_link2base_link = ee_link2base_link * tcp_link2ee_link
    // ee_link2base_link = tcp_link2base_link * (tcp_link2ee_link)^-1
    // base_link2tcp_link = T
    // tcp_link2ee_link = [1 0 0 tcp_length; 0 1 0 0; 0 0 1 0; 0 0 0 1]
    T[3] -= tcp_length*T[0];
    T[7] -= tcp_length*T[4];
    T[11] -= tcp_length*T[8];

    double q_sols[8*6], min = 1e6, dist = 0;
    int num_sols = ur_kinematics::inverse(T, q_sols), index = 0;
    for (int i = 0; i < num_sols; ++i) {
 
      for (int j = 0; j < 6; ++j) {
        q_sols[i*6 + j] = validAngle(q_sols[i*6 + j]);
        dist += pow(q_sols[i*6 + j] - joint_[j], 2);
      }
      // Take the nearest one
      if (min > dist) {
        min = dist;
        index = i;
      } dist = 0;
    }

    for (int i = 0; i < 6; ++i)
      sol[i] = q_sols[index*6 + i];
  }
 
 public:
  RobotArm() : joint_() {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("/follow_joint_trajectory", true);

    // wait for action server to come up
    while (!traj_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the joint_trajectory_action server");
      break;
    }

    octomap_client_ = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");
    exit_ = false;
    th_ = new std::thread(&RobotArm::clearOctomap, this);

    // Subscribe to /joint_state
    sub_joint_state_ = nh_.subscribe("/joint_states", 1, &RobotArm::jointStateCallback, this);
    // Subscribe to /tag_detections
    sub_tag_id_ = nh_.subscribe("/tag_detections", 1, &RobotArm::apriltagCallback, this);

    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    t.joint_names.resize(6);
    t.joint_names[0] = "shoulder_pan_joint";
    t.joint_names[1] = "shoulder_lift_joint";
    t.joint_names[2] = "elbow_joint";
    t.joint_names[3] = "wrist_1_joint";
    t.joint_names[4] = "wrist_2_joint";
    t.joint_names[5] = "wrist_3_joint";
  }
  ~RobotArm() {
    exit_ = true;
    th_->join();
    delete th_;
    delete traj_client_;
  }

  void clearOctomap() {
  	std_srvs::Empty srv;
  	while(!exit_) {
  	  usleep(1000000);
  	  octomap_client_.call(srv);
  	}
  }

  // Start to execute the action
  // Input: goal
  // Output: None
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal) {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now();
    traj_client_->sendGoal(goal);
  }
  // Let arm to standby position (predefined)
  // Input: None
  // Output: goal
  control_msgs::FollowJointTrajectoryGoal armStandbyTrajectory() {
    ros::spinOnce();
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    t.points.resize(2);
    for (int i = 0; i < 2; ++i) {
      t.points[i].positions.resize(6);
      t.points[i].velocities.resize(6);      
    }
    t.points[0].time_from_start = ros::Duration(0);
    // Added by Sean
    // Calculate the execution time
    double dist = 0;
    for(int i=0; i<6; ++i)
      dist += pow(joint_[i] - joint_standby[i], 2);
    dist = sqrt(dist);
    int cost_time = ceil(dist / joint_speed)* 3; // Multiply 3 to slow down
    t.points[1].time_from_start = ros::Duration(cost_time);
    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint_[i];
      t.points[0].velocities[i] = 0;
      t.points[1].positions[i] = joint_standby[i];
      t.points[1].velocities[i] = 0;
    }
    return goal_;
  }
  // Added by Sean
  // Let arm to shelf position (predefined)
  // Input: None
  // Output: goal
  control_msgs::FollowJointTrajectoryGoal armShelfTrajectory() {
    ros::spinOnce();
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    t.points.resize(2);
    for (int i = 0; i < 2; ++i) {
      t.points[i].positions.resize(6);
      t.points[i].velocities.resize(6);      
    }
    t.points[0].time_from_start = ros::Duration(0);
    // Added by Sean
    // Calculate the execution time
    double dist = 0;
    for(int i=0; i<6; ++i)
      dist += pow(joint_[i] - joint_standby[i], 2);
    dist = sqrt(dist);
    int cost_time = ceil(dist / joint_speed)* 3; // Multiply 3 to slow down
    t.points[1].time_from_start = ros::Duration(cost_time);

    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint_[i];
      t.points[0].velocities[i] = 0;
      t.points[1].positions[i] = joint_shelf[i];
      t.points[1].velocities[i] = 0;
    }
    return goal_;
  }
  // Let arm to desired pose
  // Input: pose
  // Output: goal
  control_msgs::FollowJointTrajectoryGoal armToDesiredPoseTrajectory(geometry_msgs::Pose pose) {
    ros::spinOnce();
    double sol[6] = {0};
    //pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.5708, 0.707, 1.5708);
    perform_IK(pose, sol);
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    t.points.resize(2);
    for (int i = 0; i < 2; ++i) {
      t.points[i].positions.resize(6);
      t.points[i].velocities.resize(6);
    }

    t.points[0].time_from_start = ros::Duration(0);
    // Added by Sean.
    // Calculate the execution time
    double dist = 0;
    for(int i=0; i<6;++i){
      dist += pow(sol[i]-joint_[i], 2);
    }
    dist = sqrt(dist);
    int cost_time = ceil(dist/ joint_speed)* 3; // Multiply 3 to slow down
    t.points[1].time_from_start = ros::Duration(cost_time);

    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint_[i];
      t.points[0].velocities[i] = 0;
      t.points[1].positions[i] = sol[i];
      t.points[1].velocities[i] = 0;
    }
    return goal_;
  }
  // Get execution state
  // Input: None
  // Output: state
  actionlib::SimpleClientGoalState getState() {
    ros::spinOnce();
    return traj_client_->getState();
  }
  // Added by Sean
  // Input: pose
  // Output: 1 if the pose is in the working area and 0 otherwise
  bool getNumofSols(geometry_msgs::Pose pose) {
    double T[16] = {0};
    pose_to_DH(pose, T);
    T[3] -= tcp_length*T[0];
    T[7] -= tcp_length*T[4];
    T[11] -= tcp_length*T[8];

    double q_sols[8*6];
    int num_sols = ur_kinematics::inverse(T, q_sols);
    return (num_sols >= 1);
  }
}; // end class definition

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "UR5_action");
  ros::NodeHandle nh;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  // Publisher for robotiq gripper command
  //ros::Publisher pub_gripper_cmd;
  //pub_gripper_cmd = nh.advertise<robotiq_c_model_control::CModel_robot_output>("CModelRobotOutput",1);
  // Gripper command
  //robotiq_c_model_control::CModel_robot_output cmd;
  // Open the gripper for initial
  //cmd.rACT = 1; cmd.rGTO = 1; cmd.rATR = 0; cmd.rPR = 0; cmd.rSP = 255; cmd.rFR= 40;
  //pub_gripper_cmd.publish(cmd); ros::Duration(0.5).sleep();
  
  RobotArm arm;  

  char in; // waitkey

  geometry_msgs::Pose pose; // target pose

  ros::Time now = ros::Time::now();

  while (ros::Time::now().toSec() < (now.toSec() + 5)) {
    ros::spinOnce();
  }
  
  // 1. Go to the standby point
  std::cout << "Go to the standby point" << std::endl;
  // Start the trajectory
  arm.startTrajectory(arm.armStandbyTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok()) {
    usleep(500000);
  }
  usleep(1000000); // sleep one second to make sure not listen to the old tag_link frame
  ros::spinOnce();
  // 2.
  // 2.1 Pick the object
  
  try{
    listener.waitForTransform("base_link", "tag_link", ros::Time(0), ros::Duration(5.0));
    listener.lookupTransform("base_link", "tag_link", ros::Time(0), transform);
  }
  catch(tf::TransformException &ex){
    ROS_ERROR("%s", ex.what());
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
  
  std::cout << "x: " << transform.getOrigin().x() << ", y: " << transform.getOrigin().y() << ", z: " << transform.getOrigin().z() << std::endl;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();
  pose.orientation.x = transform.getRotation().x();
  pose.orientation.y = transform.getRotation().y();
  pose.orientation.z = transform.getRotation().z();
  pose.orientation.w = transform.getRotation().w();
  std::cout << "Target pose: " << std::endl << pose << std::endl;

  std::cout << "Waitkey: s to start" << std::endl;
  std::cin >> in;
  if(in != 's') return 0;
  std::cout << "Start to approach the object" << std::endl;
  if(arm.getNumofSols(pose))
    arm.startTrajectory(arm.armToDesiredPoseTrajectory(pose));
  else {
    std::cout << "Pose out of working area, emergency stop!" << std::endl;
    return 0; // End process
  }
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok()) {
    usleep(500000);
  }
  
  std::cout << "Waitkey: s to start" << std::endl;
  std::cin >> in;
  if (in != 's') return 0;
  //cmd.rACT = 1; cmd.rGTO = 1; cmd.rATR = 0; cmd.rPR = 255; cmd.rSP = 255; cmd.rFR= 40;
  //pub_gripper_cmd.publish(cmd); ros::Duration(0.5).sleep();
  // 2.2 Return to standby point
  std::cout << "return to standby point" << std::endl;
  arm.startTrajectory(arm.armStandbyTrajectory());
  while(!arm.getState().isDone() && ros::ok()) {
    usleep(500000);
  }
  // 3. Go to shelf point
  std::cout << "Go to shelf point" << std::endl;
  arm.startTrajectory(arm.armShelfTrajectory());
  while(!arm.getState().isDone() && ros::ok()) {
    usleep(500000);
  }
  usleep(1000000); // sleep one second to make sure not listen to the old tag_link frame
  ros::spinOnce();
  // 4.
  // 4.1 Place the object
  try{
    listener.waitForTransform("base_link", "tag_link", ros::Time(0), ros::Duration(5.0));
    listener.lookupTransform("base_link", "tag_link", ros::Time(0), transform);
  }
  catch(tf::TransformException &ex){
    ROS_ERROR("%s", ex.what());
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
  //std::cout << "Apriltag id:" << tag_id << " at:";
  std::cout << "x: " << transform.getOrigin().x() << ", y: " << transform.getOrigin().y() << ", z: " << transform.getOrigin().z() << std::endl;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z()+0.03;
  pose.orientation.x = transform.getRotation().x();
  pose.orientation.y = transform.getRotation().y();
  pose.orientation.z = transform.getRotation().z();
  pose.orientation.w = transform.getRotation().w();
  std::cout << "Start to place the object" << std::endl;
  std::cout << "Waitkey: s to start" << std::endl;
  std::cin >> in;
  if (in != 's') return 0;
  if(arm.getNumofSols(pose))
    arm.startTrajectory(arm.armToDesiredPoseTrajectory(pose));
  else {
    std::cout << "Pose out of working area, emergency stop!" << std::endl;
    return 0; // End process
  }
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok()) {
    usleep(500000);
  }
  //cmd.rACT = 1; cmd.rGTO = 1; cmd.rATR = 0; cmd.rPR = 0; cmd.rSP = 255; cmd.rFR= 40;
  //pub_gripper_cmd.publish(cmd); ros::Duration(0.5).sleep();
  usleep(500000); // After loose the gripper, hold 0.5s
  // 6.2 Return to shelf point
  std::cout << "Return to shelf point" << std::endl;
  arm.startTrajectory(arm.armShelfTrajectory());
  while(!arm.getState().isDone() && ros::ok()) {
    usleep(500000);
  }
  std::cout << "End process" <<std::endl;  
  // For debug
  bool debug = false;
  if(debug) {
    while(ros::ok())
    {
      ros::spinOnce();
    }
  }
  return 0;
}
