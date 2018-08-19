#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

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

// PCL Visualizer
#include <pcl/visualization/pcl_visualizer.h>

static const std::string OPENCV_WINDOW = "Image window";

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


int vis_on = 0;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

class pointcloud_processing
{

  ros::NodeHandle nh;
  ros::NodeHandle nh_pub;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, &pointcloud_processing::callback,this);
  ros::Publisher pub = nh_pub.advertise<PointCloud> ("/denoise_PointCloud", 1);

  public:
  void callback(const PointCloud::ConstPtr& msg)
  {
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZRGB& pt, msg->points)
      {
      //uint32_t rgb = *reinterpret_cast<int*>(&pt.rgb);
      uint8_t r = int(pt.r);
      uint8_t g = int(pt.g);
      uint8_t b = int(pt.b);
      printf ("\t(%f, %f, %f, %d, %d, %d)\n", pt.x, pt.y, pt.z, r, g, b);
      break;
     }
    denoise_PointCloud(msg);
  }

  void denoise_PointCloud(const PointCloud::ConstPtr& msg)
  {

  PointCloud::Ptr cloud_renan (new PointCloud);
  *cloud_renan = *msg;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_renan, *cloud_renan, indices);

  PointCloud::Ptr cloud_filtered (new PointCloud);

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_renan);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *msg << std::endl;

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;


  PointCloud::Ptr msg_pub (new PointCloud);
  msg_pub->header.frame_id = "camera_rgb_optical_frame";
  msg_pub->height = cloud_filtered->height;
  msg_pub->width = cloud_filtered->width;
  msg_pub->points = cloud_filtered->points;
  pcl_conversions::toPCL(ros::Time::now(), msg_pub->header.stamp);
  pub.publish (msg_pub);

  if (vis_on==0){
    calculate_surface_normal(cloud_filtered);
    vis_on+=1 ;
    }
  }


  void calculate_surface_normal(const PointCloud::ConstPtr& cloud_filtered)
  {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.05);

    // Compute the features
    ne.compute (*cloud_normals);

    std::cout << cloud_normals << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = normalsVis(cloud_filtered, cloud_normals);



  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
  {
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 100, 0.05, "normals");
    //viewer->addCoordinateSystem (0.1);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return (viewer);
  }


};






int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl");

  ImageConverter ic;
  pointcloud_processing pcp;
  ros::spin();
  return 0;
}
