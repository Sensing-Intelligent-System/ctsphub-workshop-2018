#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

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

// sync
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, PointCloud> SyncPolicy;

static const std::string OPENCV_WINDOW = "Image window";


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

void denoise_PointCloud(const PointCloud::ConstPtr& msg)
{
  ros::NodeHandle nh_pub;
  ros::Publisher pub = nh_pub.advertise<PointCloud> ("/Segmented_PointCloud", 1);

  PointCloud::Ptr cloud_filtered (new PointCloud);

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (msg);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_filtered);

  //std::cerr << "Cloud before filtering: " << std::endl;
  //std::cerr << *msg << std::endl;

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

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

    PointCloud::Ptr msg_pub (new PointCloud);
    msg_pub->header.frame_id = "camera_rgb_optical_frame";
    msg_pub->height = cloud_filtered->height;
    msg_pub->width = cloud_filtered->width;
    msg_pub->points = cloud_filtered->points;
    pcl_conversions::toPCL(ros::Time::now(), msg_pub->header.stamp);
    pub.publish (msg_pub);
  }

}


void callback(const sensor_msgs::ImageConstPtr& image, const PointCloud::ConstPtr& pc)
{

  //Denosie & Remove points from PointCloud
  denoise_PointCloud(pc);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "segmentation");

  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_raw", 5);
  message_filters::Subscriber<PointCloud> pc_sub(nh, "/camera/depth_registered/points", 5);

  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(5), image_sub, pc_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::spin();
  return 0;
}
