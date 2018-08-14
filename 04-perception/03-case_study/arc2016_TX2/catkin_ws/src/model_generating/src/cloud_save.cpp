#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>  
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <Eigen/Geometry>
using namespace pcl;
PointCloud<PointXYZRGB>::Ptr scene_cloud(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr scene_cloud_1(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr scene_cloud_2(new PointCloud<PointXYZRGB>);
int main(int argc, char** argv){
    /////////////////Ros node initialization////////
    ros::init(argc, argv, "cloud_icp_demo");
    ros::Time::init();
    ros::NodeHandle nh;
	sensor_msgs::PointCloud2 pc,pc1,pc2;
	pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", ros::Duration(10)));
	pc1  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", ros::Duration(10)));
	pc2  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", ros::Duration(10)));
	pcl::fromROSMsg (pc,*scene_cloud);
	pcl::fromROSMsg (pc1,*scene_cloud_1);
	pcl::fromROSMsg (pc2,*scene_cloud_2);
	*scene_cloud += *scene_cloud_1;
	*scene_cloud += *scene_cloud_2;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*scene_cloud, *scene_cloud, indices);

	// apriltags_ros::AprilTagDetectionArray tag_info;
	// tag_info  = *(ros::topic::waitForMessage<apriltags_ros::AprilTagDetectionArray>("/tag_detections", ros::Duration(10)));
 //    geometry_msgs::PoseStamped pose = tag_info.detections[0].pose;

 //    Eigen::Matrix3f mat3 = Eigen::Quaternionf(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z).toRotationMatrix(); 
 //    Eigen::Matrix3f mat3_inverse = mat3.inverse();
 //    Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity(); 
 //    mat4.block(0,0,3,3) = mat3_inverse;
 //    mat4(0,3) = -1.0 * pose.pose.position.x;
 //    mat4(1,3) = -1.0 * pose.pose.position.y;
 //    mat4(2,3) = -1.0 * pose.pose.position.z;

 //    Eigen::Matrix4f mat5 = Eigen::Matrix4f::Identity(); 
 //    mat5.block(0,0,3,3) = mat3;
 //    mat5(0,3) = pose.pose.position.x;
 //    mat5(1,3) = pose.pose.position.y;
 //    mat5(2,3) = pose.pose.position.z;


    // std::cout << mat5 << std::endl ;


    // pcl::transformPointCloud (*scene_cloud, *scene_cloud, mat4);
     pcl::io::savePLYFile ("/home/nvidia/arc2016_TX2/catkin_ws/src/icp_pose_estimation/src/model/bins/tote1.ply", *scene_cloud,true);
	ros::Publisher model_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/model", 1);
	
	while (1){
		model_publisher.publish(scene_cloud);
	}

    return 0;
}
