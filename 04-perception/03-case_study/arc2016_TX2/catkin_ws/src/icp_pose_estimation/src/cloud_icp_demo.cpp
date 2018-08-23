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
using namespace pcl;
using namespace std;
using namespace cv;
const int viva_mask_color[3] = {0,0,128};
const int kleenex_mask_color[3] = {128,128,0};
const int crayola_mask_color[3] = {0,128,0}; 
vector <string> object_list;
vector < PointCloud<PointXYZRGB>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZRGB>::Ptr > > modelClouds;
PointCloud<PointXYZRGB>::Ptr toteModel(new PointCloud<PointXYZRGB>);
ros::Publisher model_publisher; 
ros::Publisher original_object_publisher; 
ros::Publisher downsampled_object_publisher; 
ros::Publisher denoised_object_publisher; 
ros::Publisher object_publisher; 
ros::Publisher align_object_publisher; 
ros::Publisher bounding_box_publisher ;
PointCloud<PointXYZRGB>::Ptr scene_cloud(new PointCloud<PointXYZRGB>);
//////////////////For Visualization/////////////////////////////////////////
PointCloud<PointXYZRGB>::Ptr original_cloud(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr downsampled_cloud(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr denoised_cloud(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr preprocessed_cloud(new PointCloud<PointXYZRGB>);


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


vector<double> point_2_plane_icp (PointCloud<PointXYZRGB>::Ptr sourceCloud, PointCloud<PointXYZRGB>::Ptr targetCloud, PointCloud<PointXYZRGBNormal>::Ptr cloud_source_trans_normals ){
  	pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_source_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
  	pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_target_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
  	pcl::PointCloud<PointXYZRGB>::Ptr translated_sourceCloud(new pcl::PointCloud<PointXYZRGB>);
  	Eigen::Matrix4f transform_translation = Eigen::Matrix4f::Identity();
  	Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*sourceCloud, centroid);
    transform_translation(0,3) -= centroid[0];
    transform_translation(1,3) -= centroid[1];
    transform_translation(2,3) -= centroid[2];
    pcl::transformPointCloud (*sourceCloud, *translated_sourceCloud, transform_translation);
    
    printf("Test1\n");
  	addNormal( translated_sourceCloud, cloud_source_normals );
    printf("Test2\n");
  	addNormal( targetCloud, cloud_target_normals );
  	// addNormal( cloud_source_trans, cloud_source_trans_normals );
  	pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );
  	icp->setMaximumIterations ( 200 );
  	// icp.setMaxCorrespondenceDistance(1);  
   //  icp.setTransformationEpsilon(1e-10);  
   //  icp.setEuclideanFitnessEpsilon(0.01);  
  	icp->setInputSource ( cloud_source_normals ); // not cloud_source, but cloud_source_trans!
  	icp->setInputTarget ( cloud_target_normals );
  
   	// registration
  	icp->align ( *cloud_source_trans_normals ); // use cloud with normals for ICP
    
  	if ( icp->hasConverged() ){
   		std::cout << "icp score: " << icp->getFitnessScore() << std::endl;
      // std::cout << icp->getFinalTransformation() << std::endl;
   	}
   	else
   		std::cout << "Not converged." << std::endl;
    //The Transformation from model to object cloud


    ///////////Generate the transform matrix from model to object scene
    Eigen::Matrix4f inverse_transformation = icp->getFinalTransformation();
    Eigen::Matrix3f inverse_object_rotation_matrix;
    for(int row=0;row<3;row++){
      for(int col=0;col<3;col++)
        inverse_object_rotation_matrix(row,col) = inverse_transformation(row,col);
    }
    Eigen::Matrix3f object_rotation_matrix = inverse_object_rotation_matrix.inverse();
    Eigen::Matrix4f object_transform_matrix = Eigen::Matrix4f::Identity(); 
    for(int row=0;row<3;row++){
      object_transform_matrix(row,3) = -1.0 * inverse_transformation(row,3);
      for(int col=0;col<3;col++)
        object_transform_matrix(row,col) = object_rotation_matrix(row,col);
    }

    // std::cout << object_transform_matrix << std::endl;
    object_transform_matrix(0,3) += centroid[0];
    object_transform_matrix(1,3) += centroid[1];
    object_transform_matrix(2,3) += centroid[2];

  	tf::Matrix3x3 tf3d;
  	tf3d.setValue((object_transform_matrix(0,0)), (object_transform_matrix(0,1)), (object_transform_matrix(0,2)), 
        (object_transform_matrix(1,0)), (object_transform_matrix(1,1)), (object_transform_matrix(1,2)), 
        (object_transform_matrix(2,0)), (object_transform_matrix(2,1)), (object_transform_matrix(2,2)));
  	tf::Quaternion tfqt;
  	tf3d.getRotation(tfqt);
  	vector<double> rot_and_tra;
  	rot_and_tra.resize(7);
  	rot_and_tra[0]=tfqt[0];//euler_angle[0]; 
  	rot_and_tra[1]=tfqt[1];//euler_angle[1]; 
  	rot_and_tra[2]=tfqt[2];//euler_angle[2]; 
  	rot_and_tra[3]=tfqt[3];//euler_angle[2]; 
  	rot_and_tra[4]=object_transform_matrix(0,3);
  	rot_and_tra[5]=object_transform_matrix(1,3);
  	rot_and_tra[6]=object_transform_matrix(2,3);
	
	  return rot_and_tra;
}

void update_points(const sensor_msgs::PointCloud2::ConstPtr& cloud){
	  pcl::fromROSMsg (*cloud,*scene_cloud);
  	return;
}

void icp_point_cloud_preprocessing(PointCloud<PointXYZRGB>::Ptr object_cloud){
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*object_cloud, *object_cloud, indices);
  
  //////////////Pointcloud downsampling////////////////////
  pcl::VoxelGrid<PointXYZRGB> sor;
  sor.setInputCloud (object_cloud);
  sor.setLeafSize (0.002f, 0.002f, 0.002f);
  sor.filter (*object_cloud);  
  copyPointCloud(*object_cloud, *downsampled_cloud);

  //////////////Pointcloud Denoise////////////////////
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
  if (object_cloud->points.size()>100){
    sor2.setInputCloud (object_cloud);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (0.5);
    sor2.filter (*object_cloud);
  }

  std::vector<int> indices2;
  pcl::removeNaNFromPointCloud(*object_cloud, *object_cloud, indices2);
  copyPointCloud(*object_cloud, *denoised_cloud);
  return;
}

void background_align (PointCloud<PointXYZRGB>::Ptr background_model,PointCloud<PointXYZRGB>::Ptr scene_clouds, PointCloud<PointXYZRGB>::Ptr background_clouds){
    // Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    // initial_guess(0,0) = 0.0293716; initial_guess(0,1) = 0.999368; initial_guess(0,2) = 0.020094; initial_guess(0,3) = 0.000858; 
    // initial_guess(1,0) = 0.980191; initial_guess(1,1) = -0.0248739; initial_guess(1,2) = -0.196486; initial_guess(1,3) = 0.020039; 
    // initial_guess(2,0) = -0.195864; initial_guess(2,1) = 0.0253842; initial_guess(2,2) = -0.980302; initial_guess(2,3) = 0.671708; 
    // PointCloud<PointXYZRGB>::Ptr background_model_align (new PointCloud<PointXYZRGB>);
    // pcl::transformPointCloud (*background_model, *background_model_align, initial_guess);
    pcl::PointCloud<PointXYZRGBNormal>::Ptr background_model_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
    pcl::PointCloud<PointXYZRGBNormal>::Ptr scene_clouds_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
    pcl::PointCloud<PointXYZRGBNormal>::Ptr background_clouds_reg ( new pcl::PointCloud<PointXYZRGBNormal> );
    //////////////Pointcloud Denoise////////////////////

    addNormal( background_model, background_model_normals );
    addNormal( scene_clouds, scene_clouds_normals );
    pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );
    icp->setMaximumIterations ( 200 ); 
    icp->setInputSource ( background_model_normals ); // not cloud_source, but cloud_source_trans!
    icp->setInputTarget ( scene_clouds_normals );
  
    // registration
    icp->align ( *background_clouds_reg ); // use cloud with normals for ICP
    pcl::copyPointCloud(*background_clouds_reg,*background_clouds);
    if ( icp->hasConverged() ){
      std::cout << "icp score: " << icp->getFitnessScore() << std::endl;
      // std::cout << icp->getFinalTransformation() << std::endl;
    }
    else
      std::cout << "Not converged." << std::endl;

    
    //The Transformation from model to object cloud
    model_publisher.publish(background_model);
    object_publisher.publish(scene_clouds);
    align_object_publisher.publish(background_clouds);
    return;
}

void background_extraction(PointCloud<PointXYZRGB>::Ptr background_clouds,PointCloud<PointXYZRGB>::Ptr scene_clouds){
    if (scene_clouds->points.size()<100)
      return;
    for (int i=0;i<background_clouds->points.size();i++){
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      float radius = 0.01;
      pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
      kdtree.setInputCloud (scene_clouds);
      pcl::PointXYZRGB searchPoint;
      searchPoint.x = background_clouds->points[i].x;
      searchPoint.y = background_clouds->points[i].y;
      searchPoint.z = background_clouds->points[i].z;

      if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        for (int j = 0; j < pointIdxRadiusSearch.size (); ++j){
          //printf("ID:%d\n",pointIdxRadiusSearch[j]);
          scene_clouds->points[ pointIdxRadiusSearch[j] ].z =  -1.0;
        }
        pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new  pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.0)));
        
        pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (scene_clouds);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter (*scene_clouds);
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*scene_clouds, *scene_clouds, indices);
        //printf("Size: %d\n",scene_clouds->points.size());
      }
      
    }
    return;
}

void icp_vis (PointCloud<PointXYZRGB>::Ptr model_point,  pcl::PointCloud<PointXYZRGBNormal>::Ptr icp_result_point,vector<double> product_pose){
	model_publisher.publish(model_point);
  original_object_publisher.publish(original_cloud);
  downsampled_object_publisher.publish(downsampled_cloud);
  denoised_object_publisher.publish(denoised_cloud);
	align_object_publisher.publish(icp_result_point);
	//Bounding box marker
	visualization_msgs::Marker marker;
  	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
  	marker.header.frame_id = "/camera_color_optical_frame";
  	marker.header.stamp = ros::Time::now();
   
  	// Set the namespace and id for this marker.  This serves to create a unique ID
  	// Any marker sent with the same namespace and id will overwrite the old one
  	marker.ns = "basic_shapes";
    marker.id = 0;
	  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  	uint32_t shape = visualization_msgs::Marker::CUBE;
  	marker.type = shape;
  	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  	marker.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  	marker.pose.position.x = product_pose[4];
  	marker.pose.position.y = product_pose[5];
  	marker.pose.position.z = product_pose[6];
  	marker.pose.orientation.x = product_pose[0];
  	marker.pose.orientation.y = product_pose[1];
  	marker.pose.orientation.z = product_pose[2];
  	marker.pose.orientation.w = product_pose[3]; 
  	// Set the scale of the marker -- 1x1x1 here means 1m on a side
  	PointXYZRGB minPt, maxPt;
	pcl::getMinMax3D (*model_point, minPt, maxPt);

  	marker.scale.x = maxPt.x - minPt.x;
  	marker.scale.y = maxPt.y - minPt.y;
  	marker.scale.z = maxPt.z - minPt.z; 
  	// Set the color -- be sure to set alpha to something non-zero!
  	marker.color.r = 0.0f;
  	marker.color.g = 1.0f;
  	marker.color.b = 0.0f;
  	marker.color.a = 0.3;
  	bounding_box_publisher.publish(marker);
	return ;
}

void object_cloud_filtering(PointCloud<PointXYZRGB>::Ptr cloud ,cv_bridge::CvImagePtr mask,string object){
	////////////////Decide which object to filter the cloud////////////
	if (cloud->points.size()!= (640*480)){
		printf("Weird\n");
		return;
	}


	int mask_color[3];
	if (object == "dove")
		{
		mask_color[0] = 255;
		mask_color[1] = 255;
		mask_color[2] = 255;
	}
	/*
	if (object == "viva"){
		mask_color[0] = viva_mask_color[0];
		mask_color[1] = viva_mask_color[1];
		mask_color[2] = viva_mask_color[2];
	}
	else if(object == "kleenex"){
		mask_color[0] = kleenex_mask_color[0];
		mask_color[1] = kleenex_mask_color[1];
		mask_color[2] = kleenex_mask_color[2];
	}
	else if(object == "crayola"){
		mask_color[0] = crayola_mask_color[0];
		mask_color[1] = crayola_mask_color[1];
		mask_color[2] = crayola_mask_color[2];
	}
	*/

	////////////////Filter object cloud///////////////////
	int count = 0;
	int threshold = 20;
	for (int row=0;row<480;row++){
		for(int column=0;column<640;column++){	
			if (object == "viva"){
				if	(mask->image.at<Vec3b>(row,column)[0]!=0 | mask->image.at<Vec3b>(row,column)[1]!=0 | mask->image.at<Vec3b>(row,column)[2]<threshold){
					cloud->points[count].x= std::numeric_limits<float>::quiet_NaN();
					cloud->points[count].y= std::numeric_limits<float>::quiet_NaN();
					cloud->points[count].z= std::numeric_limits<float>::quiet_NaN();
				}
			}
			if (object == "kleenex"){
				if	(mask->image.at<Vec3b>(row,column)[0]<threshold| mask->image.at<Vec3b>(row,column)[1]<threshold | mask->image.at<Vec3b>(row,column)[2]!=0){
					cloud->points[count].x= std::numeric_limits<float>::quiet_NaN();
					cloud->points[count].y= std::numeric_limits<float>::quiet_NaN();
					cloud->points[count].z= std::numeric_limits<float>::quiet_NaN();
				}
			}
			if (object == "crayola"){
				if	(mask->image.at<Vec3b>(row,column)[0]!=0| mask->image.at<Vec3b>(row,column)[1]<threshold | mask->image.at<Vec3b>(row,column)[2]!=0){
					cloud->points[count].x= std::numeric_limits<float>::quiet_NaN();
					cloud->points[count].y= std::numeric_limits<float>::quiet_NaN();
					cloud->points[count].z= std::numeric_limits<float>::quiet_NaN();
				}
			}

			if (object == "dove"){
				if	(mask->image.at<Vec3b>(row,column)[0]< threshold | mask->image.at<Vec3b>(row,column)[1]< threshold | mask->image.at<Vec3b>(row,column)[2]< threshold){
					cloud->points[count].x= std::numeric_limits<float>::quiet_NaN();
					cloud->points[count].y= std::numeric_limits<float>::quiet_NaN();
					cloud->points[count].z= std::numeric_limits<float>::quiet_NaN();
				}
			}
			count++;
		}
	}

	//////////////Remove NAN/////////////////////////////////
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  copyPointCloud(*cloud, *original_cloud);
  
	return ;
}


/////////////////////Main callback function: subscribe mask and do icp allignment to get object pose
void icp_cb(const sensor_msgs::Image::ConstPtr& mask){
  int cloud_size_thres = 100;
	cv_ptr = cv_bridge::toCvCopy(mask, sensor_msgs::image_encodings::RGB8); 

	/////////////////////////Extract background//////////////////////////////////
  PointCloud<PointXYZRGB>::Ptr tote_cloud(new PointCloud<PointXYZRGB>);
  copyPointCloud(*scene_cloud, *tote_cloud);
  icp_point_cloud_preprocessing(tote_cloud);
  printf("Size of tote point : %d\n",tote_cloud->points.size());
  PointCloud<PointXYZRGB>::Ptr tote_align_cloud(new PointCloud<PointXYZRGB>);
  if (tote_cloud->points.size()>cloud_size_thres)
    background_align(toteModel,tote_cloud,tote_align_cloud);

  ////////////////////////Generate object point cloud/////////////////////////
  PointCloud<PointXYZRGB>::Ptr crayola_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr kleenex_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr viva_cloud(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr dove_cloud(new PointCloud<PointXYZRGB>);

  copyPointCloud(*scene_cloud, *dove_cloud);
  copyPointCloud(*scene_cloud, *crayola_cloud);
  copyPointCloud(*scene_cloud, *kleenex_cloud);
  copyPointCloud(*scene_cloud, *viva_cloud);




	// ////////Dove soap box Object alignmant//////////
  object_cloud_filtering(dove_cloud,cv_ptr,"dove");
  printf("Original dove cloud size: %d\n",dove_cloud->points.size());
  icp_point_cloud_preprocessing(dove_cloud);
  background_extraction(tote_align_cloud,dove_cloud);
  printf("Downsampled and denoised dove cloud size: %d\n",dove_cloud->points.size());
	if (dove_cloud->points.size()>cloud_size_thres){
		pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_source_trans_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
		vector<double> dove_pose = point_2_plane_icp( dove_cloud,modelClouds[0],cloud_source_trans_normals);
		icp_vis(modelClouds[0],cloud_source_trans_normals,dove_pose);
  	printf("----------Finish %s ICP aligning----------\n","dove");
	}


  /*
	// ////////Viva Object alignmant//////////
  object_cloud_filtering(viva_cloud,cv_ptr,"viva");
  printf("Original Viva cloud size: %d\n",viva_cloud->points.size());
  icp_point_cloud_preprocessing(viva_cloud);
  background_extraction(tote_align_cloud,viva_cloud);
  printf("Downsampled and denoised Viva cloud size: %d\n",viva_cloud->points.size());
	if (viva_cloud->points.size()>cloud_size_thres){
		pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_source_trans_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
		vector<double> viva_pose = point_2_plane_icp( viva_cloud,modelClouds[2],cloud_source_trans_normals);
		icp_vis(modelClouds[2],cloud_source_trans_normals,viva_pose);
  	printf("----------Finish %s ICP aligning----------\n","Viva");
	}

  ////////Kleenex Object alignmant//////////
  object_cloud_filtering(kleenex_cloud,cv_ptr,"kleenex");
  printf("Original Kleenex cloud size: %d\n",kleenex_cloud->points.size());
  icp_point_cloud_preprocessing(kleenex_cloud);
  background_extraction(tote_align_cloud,kleenex_cloud);
  printf("Downsampled and denoised Kleenex cloud size: %d\n",kleenex_cloud->points.size());
	if (kleenex_cloud->points.size()>cloud_size_thres){
		pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_source_trans_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
		vector<double> kleenex_pose =point_2_plane_icp(kleenex_cloud ,modelClouds[1],cloud_source_trans_normals);
		icp_vis(modelClouds[1],cloud_source_trans_normals,kleenex_pose);
  		printf("----------Finish %s ICP aligning----------\n","Kleenex");
	}

  ////////Crayola Object alignmant//////////
  object_cloud_filtering(crayola_cloud,cv_ptr,"crayola");
  printf("Original Crayola cloud size: %d\n",crayola_cloud->points.size());
  icp_point_cloud_preprocessing(crayola_cloud);
  background_extraction(tote_align_cloud,crayola_cloud);
  printf("Downsampled and denoised Crayola cloud size: %d\n",crayola_cloud->points.size());
	if (crayola_cloud->points.size()>cloud_size_thres){
		pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_source_trans_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
		vector<double> crayola_pose =point_2_plane_icp(crayola_cloud,modelClouds[0], cloud_source_trans_normals);
		icp_vis(modelClouds[0],cloud_source_trans_normals,crayola_pose);
  		printf("----------Finish %s ICP aligning----------\n","Crayola");
	}

	*/
  	return;
}


int main(int argc, char** argv){
	
    
    //////////////////Define model path/////////////
    string object_model_path("/home/nvidia/ctsphub-workshop-2018/04-perception/03-case_study/arc2016_TX2/catkin_ws/src/icp_pose_estimation/src/model/objects/");
    string bin_model_path("/home/nvidia/ctsphub-workshop-2018/04-perception/03-case_study/arc2016_TX2/catkin_ws/src/icp_pose_estimation/src/model/bins/");
    //////////////////Load Tote Clouds//////////////
    string tote_path = bin_model_path +"tote1.ply";
    io::loadPLYFile<PointXYZRGB>(tote_path, *toteModel);
    toteModel->header.frame_id = "/camera_color_optical_frame";
    pcl::VoxelGrid<PointXYZRGB> so;
    so.setInputCloud (toteModel);
    so.setLeafSize (0.003f, 0.003f, 0.003f);
    so.filter (*toteModel);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
    sor2.setInputCloud (toteModel);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (0.5);
    sor2.filter (*toteModel);
    printf("Model cloud size: %d\n",toteModel->points.size());
    //////////////////Create object list////////////

    object_list.push_back("dove_beauty_bar"); 
    //object_list.push_back("crayola_24_ct");
    //object_list.push_back("kleenex_tissue_box");
    //object_list.push_back("kleenex_paper_towels");
    // object_list.push_back("folgers_classic_roast_coffee");

    /////////////////Create object Pointcloud list//
    for (int i = 0; i < (object_list.size()); i++)
    {
        PointCloud<PointXYZRGB>::Ptr sourceCloud(new PointCloud<PointXYZRGB>);
        string model_path = object_model_path + object_list[i] + ".ply";
        io::loadPLYFile<PointXYZRGB>(model_path, *sourceCloud);
        sourceCloud->header.frame_id = "/camera_color_optical_frame";
        pcl::VoxelGrid<PointXYZRGB> sor;
  		  sor.setInputCloud (sourceCloud);
  		  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  		  sor.filter (*sourceCloud);	
        printf("%s Model cloud size: %d\n",object_list[i].c_str(),sourceCloud->points.size());
        modelClouds.push_back(sourceCloud);
    }
    printf("-----------Finish load model clouds----------\n");

    /////////////////Ros node initialization////////
    ros::init(argc, argv, "cloud_icp_demo");
    ros::Time::init();
    ros::NodeHandle nh;
    /////////////////Declare Ros publisher and subscriber////////
    model_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/model", 1);
    original_object_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/original_object", 1); 
    downsampled_object_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/downsampled_object", 1);
    denoised_object_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/denoised_object", 1); 
    object_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/object", 1);
    align_object_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/align_object", 1);
    bounding_box_publisher= nh.advertise<visualization_msgs::Marker>("/camera/bounding_box", 1);
    ros::Subscriber object_mask_sub = nh.subscribe("mask_prediction", 1, icp_cb);
    ros::Subscriber scene_cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, update_points);

	  ros::spin();
	

    return 0;
}
