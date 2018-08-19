#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "pcl_ros/point_cloud.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cstring>
#include <iostream> 
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
using namespace pcl;
using namespace std;
//Create Point Clouds

vector <string> object_list;
vector < PointCloud<PointXYZRGB>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZRGB>::Ptr > > modelClouds;
vector < PointCloud<PointXYZRGB>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZRGB>::Ptr > > binClouds;
cv_bridge::CvImagePtr cv_ptr;
void addNormal(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );
  pcl::search::KdTree<PointXYZRGB>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  searchTree->setInputCloud ( cloud );
  pcl::NormalEstimation<PointXYZRGB, Normal> normalEstimator;
  normalEstimator.setInputCloud ( cloud );
  normalEstimator.setSearchMethod ( searchTree );
  normalEstimator.setKSearch ( 15 );
  normalEstimator.compute ( *normals );
  pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
}

void point_2_plane_icp (PointCloud<PointXYZRGB>::Ptr sourceCloud, PointCloud<PointXYZRGB>::Ptr targetCloud){
  	pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_source_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
  	pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_target_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
  	pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_source_trans_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
  	addNormal( sourceCloud, cloud_source_normals );
  	addNormal( targetCloud, cloud_target_normals );
  	printf("Finish calculating normal\n");
  	// addNormal( cloud_source_trans, cloud_source_trans_normals );
  	pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );
  	icp->setMaximumIterations ( 1000 );
  	icp->setInputSource ( cloud_source_normals ); // not cloud_source, but cloud_source_trans!
  	icp->setInputTarget ( cloud_target_normals );
  
   	// registration
  	icp->align ( *cloud_source_trans_normals ); // use cloud with normals for ICP
    
  	if ( icp->hasConverged() ){
   //  	// use cloud without normals for visualizatoin
   //  	pcl::transformPointCloud ( *cloud_source, *cloud_source_trans, icp->getFinalTransformation() );
   		std::cout << icp->getFitnessScore() << std::endl;
   	}
   	else
   		std::cout << "Not converged." << std::endl;
  
	return ;
}

void icp_cb(const sensor_msgs::PointCloud2::ConstPtr& scene_point){
	printf("Subscribe scene point\n");
	// new cloud formation 
	PointCloud<PointXYZRGB>::Ptr cloud (new PointCloud<PointXYZRGB>);
	// new pcl::cloud from  sensormsg::cloud
	pcl::fromROSMsg (*scene_point, *cloud);
	printf("Finish converting from ros message to PCL cloud\n");
  	point_2_plane_icp(cloud, binClouds[1]);
  	return;
}


int main(int argc, char** argv){
	
    PointCloud<PointXYZRGB>::Ptr toteCloud(new PointCloud<PointXYZRGB>);
    string object_model_path("/home/michael/apc-vision-toolbox/ros-packages/catkin_ws/src/pose_estimation/src/models/objects/");
    string bin_model_path("/home/michael/apc-vision-toolbox/ros-packages/catkin_ws/src/pose_estimation/src/models/bins/");
    //Create object list
    object_list.push_back("kleenex_tissue_box");
    object_list.push_back("dove_beauty_bar");

    //Create object Pointcloud list
    for (int i = 0; i < (object_list.size() - 1); i++)
    {
        PointCloud<PointXYZRGB>::Ptr sourceCloud(new PointCloud<PointXYZRGB>);
        string model_path = object_model_path + object_list[i] + ".ply";
        io::loadPLYFile<PointXYZRGB>(model_path, *sourceCloud);
        modelClouds.push_back(sourceCloud);
    }
    //Create bin Pointcloud list
    string binIds = "ABCDEFGHIJKL";
    for (int i = 0; i < binIds.length(); i++)
    {
        PointCloud<PointXYZRGB>::Ptr sourceCloud(new PointCloud<PointXYZRGB>);
        string bin_path = bin_model_path  + "bin" + binIds[i] +".ply";
        io::loadPLYFile<PointXYZRGB>(bin_path, *sourceCloud);
        sourceCloud->header.frame_id="/camera";
        binClouds.push_back(sourceCloud);
    }
    //Create tote Pointcloud
    string tote_path = bin_model_path +"tote.ply";
    io::loadPLYFile<PointXYZRGB>(tote_path, *toteCloud);
    toteCloud->header.frame_id = "/camera";
    printf("Bin point numbers: %d\n",toteCloud->points.size());

    ros::init(argc, argv, "cloud_icp_align");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	ros::Publisher tote_model_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/background_model", 1);
	sensor_msgs::PointCloud2 tote_cloud_ros;
	while (ros::ok()){
		pcl::toROSMsg(*toteCloud,tote_cloud_ros );
		tote_model_publisher.publish(binClouds[1]);
		loop_rate.sleep();
	}
	// ros::Subscriber scene_object_sub = nh.subscribe("scene_pointcloud", 1, icp_cb);
	// ros::spin();
	return 0;

}