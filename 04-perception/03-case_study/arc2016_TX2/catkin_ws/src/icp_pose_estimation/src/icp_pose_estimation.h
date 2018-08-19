#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "pcl_ros/point_cloud.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cstring>
#include <iostream> 
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf/transform_listener.h>
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
#include <geometry_msgs/PoseStamped.h>
using namespace pcl;
using namespace std;
using namespace cv;

class Icp_pose_estimation{
  public:
    Icp_pose_estimation();
  private:

    ros::Publisher model_publisher; 
    ros::Publisher original_object_publisher; 
    ros::Publisher downsampled_object_publisher; 
    ros::Publisher denoised_object_publisher; 
    ros::Publisher object_publisher; 
    ros::Publisher align_object_publisher; 
    ros::Publisher bounding_box_publisher;
    ros::Publisher product_pose_publisher;
    ros::Subscriber object_mask_sub;
    ros::Subscriber scene_cloud_sub;
    //////////////////For Visualization/////////////////////////////////////////
    vector <string> object_list;
    vector < PointCloud<PointXYZRGB>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZRGB>::Ptr > > modelClouds;
    
    PointCloud<PointXYZRGB>::Ptr toteModel;//(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr scene_cloud;//(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr original_cloud;//(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr downsampled_cloud;//(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr denoised_cloud;//(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr preprocessed_cloud;//(new PointCloud<PointXYZRGB>);
    cv_bridge::CvImagePtr cv_ptr;


    /////////////////Functions//////////////////////////
    void load_models();
    void update_points(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void icp_cb(const sensor_msgs::Image::ConstPtr& mask);
    void object_cloud_filtering(PointCloud<PointXYZRGB>::Ptr cloud ,cv_bridge::CvImagePtr mask,string object);
    void background_align (PointCloud<PointXYZRGB>::Ptr background_model,PointCloud<PointXYZRGB>::Ptr scene_clouds, PointCloud<PointXYZRGB>::Ptr background_clouds);
    void background_extraction(PointCloud<PointXYZRGB>::Ptr background_clouds,PointCloud<PointXYZRGB>::Ptr scene_clouds);
    void icp_point_cloud_preprocessing(PointCloud<PointXYZRGB>::Ptr object_cloud);
    void addNormal(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals);
    vector<double> point_2_plane_icp (PointCloud<PointXYZRGB>::Ptr sourceCloud, PointCloud<PointXYZRGB>::Ptr targetCloud, PointCloud<PointXYZRGBNormal>::Ptr cloud_source_trans_normals ); 
    void icp_vis (PointCloud<PointXYZRGB>::Ptr model_point,  pcl::PointCloud<PointXYZRGBNormal>::Ptr icp_result_point,vector<double> product_pose);
    
    
};

