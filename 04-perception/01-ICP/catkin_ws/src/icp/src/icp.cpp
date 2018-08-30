#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

// cv bridge and opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL library
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>

// sync
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, PointCloud> SyncPolicy;

static const std::string OPENCV_WINDOW = "Image window";


PointCloud::Ptr model_cloud (new PointCloud);
pcl::PLYReader Reader;

//pnode_ = ros::NodeHandlePtr(new ros::NodeHandle("~"));
ros::Publisher pub;

int RGB_process(int r, int g, int b)
{
     
    //Color filtering
    cv::Mat src(1,1,CV_8UC3,cv::Scalar::all(0));
    cv::Mat dst(1,1,CV_8UC3,cv::Scalar::all(0));
    src.at<cv::Vec3b>(0,0)[0] = b;  
    src.at<cv::Vec3b>(0,0)[1] = g;  
    src.at<cv::Vec3b>(0,0)[2] = r;  

    cvtColor(src, dst, cv::COLOR_BGR2HSV);


    // Find white box (Dove)
    int h_max = 180, h_min = 0;
    int s_max = 30, s_min = 0; 
    int v_max = 255, v_min = 130;

    for(int i = 0; i < 1; i ++)
    {
      cv::Vec3b * row_ptr = dst.ptr<cv::Vec3b>(i);
      for(int j = 0; j < 1; j ++)
      {
        cv::Vec3b hsv = row_ptr[j];
        if((int(hsv[0]) > h_max || int(hsv[0]) < h_min)  // h range
          || (int(hsv[1]) > s_max || int(hsv[1]) < s_min) // s range
          || (int(hsv[2]) > v_max || int(hsv[2]) < v_min)) // v range
        {
          row_ptr[j] = cv::Vec3b::all(0);   // remove points not in range
        }
      }
    }
  cvtColor(dst, dst, cv::COLOR_HSV2BGR);
  int is_remove = 0;
  if (dst.at<cv::Vec3b>(0,0)[0] == 0 && dst.at<cv::Vec3b>(0,0)[1] == 0 && dst.at<cv::Vec3b>(0,0)[2] ==0)
    is_remove = 1;

  return is_remove;
}


void object_pose_vis(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_norm ,Eigen::Matrix4f model2obj_tf, ros::NodeHandlePtr node_, ros::NodeHandlePtr pose_node_)
{

  Eigen::Vector4f pcaCentroid;
  pcl::PointXYZRGB  minPt; 
  pcl::PointXYZRGB  maxPt;

  pcl::compute3DCentroid(*cloud, pcaCentroid);
  pcl::getMinMax3D (*cloud, minPt, maxPt);
  Eigen::Matrix3f covariance;
  
  
  pcl::computeCovarianceMatrixNormalized(*cloud_norm, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
  Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
  transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
  transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pcaCentroid.head<3>());// 
 
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::transformPointCloud(*cloud_norm, *transformedCloud, transform);
 
  std::cout << eigenValuesPCA << std::endl;
  std::cout << eigenVectorsPCA << std::endl;
  
  Eigen::Affine3f tra_aff(transform);
  Eigen::Vector3f pz = eigenVectorsPCA.col(0);
  Eigen::Vector3f py = eigenVectorsPCA.col(1);
  Eigen::Vector3f px = eigenVectorsPCA.col(2);
  pcl::transformVector(pz, pz, tra_aff);
  pcl::transformVector(py, py, tra_aff);
  pcl::transformVector(px, px, tra_aff);



  pcl::PointXYZRGB pcaZ;
  pcaZ.x = 1000 * pz(0);
  pcaZ.y = 1000 * pz(1);
  pcaZ.z = 1000 * pz(2);
  pcl::PointXYZRGB pcaY;
  pcaY.x = 1000 * py(0);
  pcaY.y = 1000 * py(1);
  pcaY.z = 1000 * py(2);
  pcl::PointXYZRGB pcaX;
  pcaX.x = 1000 * px(0);
  pcaX.y = 1000 * px(1);
  pcaX.z = 1000 * px(2);

  tf::Matrix3x3 tf3d;
  tf3d.setValue((pcaX.x), (pcaX.y), (pcaX.z), 
        (pcaY.x), (pcaY.y), (pcaY.z), 
        (pcaZ.x), (pcaZ.y), (pcaZ.z));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);


  float x = pcaCentroid(0);
  float y = pcaCentroid(1);
  float z = pcaCentroid(2);



  tf::Matrix3x3 pose_tf3d;
  pose_tf3d.setValue((model2obj_tf(0,0)), (model2obj_tf(0,1)), (model2obj_tf(0,2)), 
        (model2obj_tf(1,0)), (model2obj_tf(1,1)), (model2obj_tf(1,2)), 
        (model2obj_tf(2,0)), (model2obj_tf(2,1)), (model2obj_tf(2,2)));
   tf::Quaternion pose_tfqt;
   pose_tf3d.getRotation(pose_tfqt);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "camera_rgb_optical_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE ;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pcaCentroid(0);;
  marker.pose.position.y = pcaCentroid(1);
  marker.pose.position.z = pcaCentroid(2);
  marker.pose.orientation.x = pose_tfqt[0];
  marker.pose.orientation.y = pose_tfqt[1];
  marker.pose.orientation.z = pose_tfqt[2];
  marker.pose.orientation.w = pose_tfqt[3];
  marker.scale.x = maxPt.x - minPt.x;
  marker.scale.y = maxPt.y - minPt.y;
  marker.scale.z = maxPt.z - minPt.z; 
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;


  //only if using a MESH_RESOURCE marker type:
  //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  ros::Publisher vis_pub = node_ -> advertise<visualization_msgs::Marker>( "obj_marker", 0 );
  vis_pub.publish( marker );
  

  geometry_msgs::PoseStamped pose_msg;

  pose_msg.header.frame_id = "camera_rgb_optical_frame";
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = z;
  pose_msg.pose.orientation.x = pose_tfqt[0];
  pose_msg.pose.orientation.y = pose_tfqt[1];
  pose_msg.pose.orientation.z = pose_tfqt[2];
  pose_msg.pose.orientation.w = pose_tfqt[3];

  ros::Publisher pose_pub = pose_node_ -> advertise<geometry_msgs::PoseStamped>("obj_pose", 0);
  pose_pub.publish(pose_msg);

}













void addNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals
)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  searchTree->setInputCloud ( cloud );

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud ( cloud );
  normalEstimator.setSearchMethod ( searchTree );
  //normalEstimator.setKSearch ( 50 );
  normalEstimator.setRadiusSearch (0.01);
  normalEstimator.compute ( *normals );
  
  pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );

  std::vector<int> indices;
  pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals, *cloud_with_normals, indices);


}





void estimate_obj_pose(PointCloud::Ptr& model_cloud, PointCloud::Ptr& obj_cloud)
{
  ros::NodeHandle nh_pub;
  ros::Publisher pub = nh_pub.advertise<PointCloud> ("/Transformed_pc", 1);

  ros::NodeHandle marker_handle;
  ros::NodeHandle pose_handle;
  ros::Publisher vis_pub = marker_handle.advertise<visualization_msgs::Marker>( "obj_marker", 0 );
  ros::Publisher pose_pub = pose_handle.advertise<geometry_msgs::PoseStamped>("obj_pose", 0);

  std::cerr << *model_cloud << std::endl;
  std::cerr << *obj_cloud << std::endl;


  // Downsample model cloud
  PointCloud::Ptr model_sample (new PointCloud);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (model_cloud);
  sor.setLeafSize (0.003f, 0.003f, 0.003f);
  sor.filter (*model_sample);

   // Downsample obj cloud
  PointCloud::Ptr obj_sample (new PointCloud);
  pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
  sor1.setInputCloud (obj_cloud);
  sor1.setLeafSize (0.002f, 0.002f, 0.002f);
  sor1.filter (*obj_sample);


  std::cerr << "model sample: " << *model_sample << std::endl;
  std::cerr << "obj sample: " <<*obj_sample << std::endl;


  // prepare could with normals
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final_ptr ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
  //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_trans_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
  pcl::PointCloud<pcl::PointXYZRGBNormal> Final;
  *final_ptr = Final;

  addNormal( model_sample, cloud_source_normals );
  addNormal( obj_sample, cloud_target_normals );
  //addNormal( cloud_source_trans, cloud_source_trans_normals );




   //icp point to point
  pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
  icp.setInputSource(cloud_source_normals); //model_sample
  icp.setInputTarget(cloud_target_normals); //obj_sample
  icp.setMaximumIterations ( 150 );
  icp.setTransformationEpsilon (1e-8);
  icp.setEuclideanFitnessEpsilon (1e-5);
  icp.setMaxCorrespondenceDistance (0.5);
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  



  /*
  // icp point to plane
  pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );
  icp->setMaximumIterations ( 150 );
  icp->setInputSource ( cloud_source_normals ); // not cloud_source, but cloud_source_trans!
  icp->setInputTarget ( cloud_target_normals );

  // registration
  icp->align ( Final ); // use cloud with normals for ICP

  std::cout << "has converged:" << icp->hasConverged() << " score: " << icp->getFitnessScore() << std::endl;
  std::cout << icp->getFinalTransformation() << std::endl;
  */



  // use cloud without normals for visualizatoin
  PointCloud::Ptr model_transformed (new PointCloud);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_transformed_norm (new  pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::transformPointCloud (*model_sample, *model_transformed, icp.getFinalTransformation() );
  pcl::transformPointCloud (*cloud_source_normals, *model_transformed_norm, icp.getFinalTransformation() );

  PointCloud::Ptr msg_pub (new PointCloud);
  msg_pub->header.frame_id = "camera_rgb_optical_frame";
  msg_pub->height = model_transformed->height;
  msg_pub->width = model_transformed->width;
  msg_pub->points = model_transformed->points;
  pcl_conversions::toPCL(ros::Time::now(), msg_pub->header.stamp);
  pub.publish (msg_pub);

  ros::NodeHandlePtr node_ (new ros::NodeHandle);
  ros::NodeHandlePtr pose_node_ (new ros::NodeHandle);
  object_pose_vis(model_transformed, model_transformed_norm, icp.getFinalTransformation(), node_, pose_node_);



}



void denoise_PointCloud(const PointCloud::ConstPtr& msg)
{
  //ros::NodeHandle nh_pub;
  //ros::Publisher pub = nh_pub.advertise<PointCloud> ("/Segmented_PointCloud", 1);

  PointCloud::Ptr cloud_renan (new PointCloud);
  PointCloud::Ptr cloud_filtered (new PointCloud);


  *cloud_renan = *msg;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_renan, *cloud_renan, indices);

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_renan);
  sor.setMeanK (50); //50
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_filtered);

  //std::cerr << "Cloud before filtering: " << std::endl;
  //std::cerr << *msg << std::endl;

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  
  //*cloud_filtered = *msg;      // ignore denoise

  int count = 0;
  if (cloud_filtered->height != 0 && cloud_filtered->width != 0)
  {


    BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud_filtered->points)
    {
      //uint32_t rgb = *reinterpret_cast<int*>(&pt.rgb);
      int is_remove = RGB_process(pt.r,pt.g,pt.b);
      if (is_remove)
      {
          count+=1;
          pt.r = 0;
          pt.g = 0;
          pt.b = 0;
          pt.x = 0;
          pt.y = 0;
          pt.z = 0;
      }
    }


    std::cout << "Remove pts: " << count << std::endl;

    PointCloud::Ptr cloud_filtered_removed (new PointCloud);

    /// Filter the cloud, Remove any points which have the coordinate Z=0
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0,0.0);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered_removed);


    PointCloud::Ptr msg_pub (new PointCloud);
    msg_pub->header.frame_id = "camera_rgb_optical_frame";
    msg_pub->height = cloud_filtered_removed->height;
    msg_pub->width = cloud_filtered_removed->width;
    msg_pub->points = cloud_filtered_removed->points;
    pcl_conversions::toPCL(ros::Time::now(), msg_pub->header.stamp);
    pub.publish (msg_pub);

    estimate_obj_pose(model_cloud, cloud_filtered_removed);
  
 }
}



void callback(const sensor_msgs::ImageConstPtr& image, const PointCloud::ConstPtr& pc)
{

  //Denosie & Remove points from PointCloud
  sleep(1);
  denoise_PointCloud(pc);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "icp");

  ros::NodeHandle nh;
 
  //Reader.read("/home/peter/ctsp_workshop/ctsphub-workshop-2018/04-perception/01-ICP/catkin_ws/src/icp/src/dove.ply", *model_cloud);
  Reader.read("/home/nvidia/ctsphub-workshop-2018/04-perception/01-ICP/catkin_ws/src/icp/src/dove.ply", *model_cloud);
  model_cloud ->header.frame_id = "/camera_color_optical_frame";

  PointCloud::Ptr cloud_filtered (new PointCloud);

  // filtering object model
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (model_cloud);
  sor.setMeanK (50); 
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_filtered);

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_raw", 5);
  message_filters::Subscriber<PointCloud> pc_sub(nh, "/camera/depth_registered/points", 5);
  pub = nh.advertise<PointCloud> ("/Segmented_PointCloud", 1);

  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(5), image_sub, pc_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::spin();
  return 0;
}
