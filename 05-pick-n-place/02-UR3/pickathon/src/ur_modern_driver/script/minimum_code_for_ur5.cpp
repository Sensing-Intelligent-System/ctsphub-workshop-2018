#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <ur_kin/ur_kin.h>
#include <std_srvs/Empty.h>
#include <thread>

const double joint_standby[6] = {-0.015878979359762013, -2.761099163685934, 1.8138976097106934, -2.067152802144186, -1.5698125998126429, -1.5917065779315394};
const double joint_home[6] = {0, -1.570724, 0, -1.570724, 0, 0};
const double tcp_length = 0.16;

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm {
 private:
  TrajClient* traj_client_;
  control_msgs::FollowJointTrajectoryGoal goal_;  
  ros::NodeHandle nh_;
  ros::Subscriber sub_joint_state_;
  ros::ServiceClient client_, octomap_client_;
  std::thread *th_;

  geometry_msgs::Quaternion fetch_pose_;
  double joint_[6];
  double position_now[6];

  double validAngle(double angle) {
    if (abs(angle) > 2*M_PI) angle = fmod(angle, 2*M_PI);
    if (angle < -M_PI) return 2*M_PI + angle;
    else if (angle > M_PI) return angle - 2*M_PI;
    else return angle;
  }
  void jointStateCallback(const sensor_msgs::JointState &msg) {
    for (int i = 0; i < 6; ++i)
      joint_[i] = msg.position[i];
  }
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
  void perform_IK(geometry_msgs::Pose target_pose, double *sol) {
    double T[16] = {0};
    // TCP's DH Matrix
    pose_to_DH(target_pose, T);
    // ee_link's DH Matrix
    //T[3] -= tcp_length*T[0];
    //T[7] -= tcp_length*T[4];
    //T[11] -= tcp_length*T[8];

    double q_sols[8*6], min = 1e6, dist = 0;
    int num_sols = ur_kinematics::inverse(T, q_sols), index = 0;
    for (int i = 0; i < num_sols; ++i) {
      // Preprocess joint angle to fit joint state msg.
      //q_sols[i*6 + 1] -= 2*M_PI;
      //q_sols[i*6 + 4] -= 2*M_PI;
 
      for (int j = 0; j < 6; ++j) {
        q_sols[i*6 + j] = validAngle(q_sols[i*6 + j]);
        dist += pow(q_sols[i*6 + j] - joint_[j], 2);
      }
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
    while (!traj_client_->waitForServer(ros::Duration(5.0))) 
      ROS_INFO("Waiting for the joint_trajectory_action server");
    
    octomap_client_ = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");

    // Subscribe to /ariac/joint_state
    sub_joint_state_ = nh_.subscribe("/joint_states", 1, &RobotArm::jointStateCallback, this);
    
    fetch_pose_.y = 
    fetch_pose_.w = 0.707;    

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
    delete traj_client_;
  }
  
  void clearOctomap() {
    std_srvs::Empty srv;
    octomap_client_.call(srv);
  }  

  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal) {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now();
    traj_client_->sendGoal(goal);
  } 
  control_msgs::FollowJointTrajectoryGoal armStandbyTrajectory() {
    ros::spinOnce();
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    t.points.resize(2); 
    for (int i = 0; i < 2; ++i) {
      t.points[i].positions.resize(6);
      t.points[i].velocities.resize(6);      
    }
    t.points[0].time_from_start = ros::Duration(0);
    t.points[1].time_from_start = ros::Duration(5);

    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint_[i];
      t.points[0].velocities[i] = 0;
      t.points[1].positions[i] = joint_standby[i];
      t.points[1].velocities[i] = 0;
    }
    return goal_;
  }
  control_msgs::FollowJointTrajectoryGoal armToDesiredPoseTrajectory(geometry_msgs::Pose pose) {
    ros::spinOnce();
    double sol[6] = {0};
    //pose.orientation = fetch_pose_;
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-1.5708, 0.0, 0.0);
    perform_IK(pose, sol);

    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    t.points.resize(2); 
    for (int i = 0; i < 2; ++i) {
      t.points[i].positions.resize(6);
      t.points[i].velocities.resize(6);
    }

    t.points[0].time_from_start = ros::Duration(0);
    // Added by Sean.
    // TODO: Design a function to calculate cost time.
    //       It should be a function w.r.t the 
    //       distance between two points.
    //double dist = sqrt(pow(pose.position.x - position_now[0], 2)+pow(pose.position.y - position_now[1], 2)+pow(pose.position.z - position_now[2], 2));
    //int cost_time = ceil(dist/ 0.5) *2;
    t.points[1].time_from_start = ros::Duration(5);

    for (int i = 0; i < 6; ++i) { 
      t.points[0].positions[i] = joint_[i];
      t.points[0].velocities[i] = 0;
      t.points[1].positions[i] = sol[i];
      t.points[1].velocities[i] = 0;
    }
    //std::cout << goal_ << std::endl;
    return goal_;
  }

  // Added by Sean.

  void get_current_position(){

    double T[16] = {0};

    ur_kinematics::forward(joint_, T);


    for (int i = 0; i < 3; ++i)

      position_now[i] = T[(i+1)*4 -1];

  }

  actionlib::SimpleClientGoalState getState() {
    ros::spinOnce();
    return traj_client_->getState();
  }
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "UR5_action");
  tf::StampedTransform transform;
  tf::TransformListener listener;

  RobotArm arm;  

  ros::Time now = ros::Time::now();

  while (ros::Time::now().toSec() < (now.toSec() + 5)) {
    ros::spinOnce();
  }

  // Lift up by 10 cm.
  geometry_msgs::Pose pose;
  pose.position.x = -0.05;
  pose.position.y = 0.13;
  pose.position.z = 0.7;
  
  std::cout << "Start the trajectory 1." << std::endl;
  // Start the trajectory
  arm.startTrajectory(arm.armStandbyTrajectory());
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok()) {
    usleep(500000);
  }

  std::cout << "Start the trajectory 2." << std::endl;
  char in;

  arm.clearOctomap();
  
  std::cout << "Waitkey: s to start" << std::endl;
  std::cin >> in ;
  std::cout << arm.armToDesiredPoseTrajectory(pose) << std::endl;
  if (in=='s'){
    arm.startTrajectory(arm.armToDesiredPoseTrajectory(pose));
    // Wait for trajectory completion
    while(!arm.getState().isDone() && ros::ok()) {
      usleep(500000);
    }
  }
  return 0;
}
