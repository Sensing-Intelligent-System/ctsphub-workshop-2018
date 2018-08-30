// ---------------------------------------------------------
// Copyright (c) 2016, Andy Zeng
// 
// This file is part of the APC Vision Toolbox and is available 
// under the terms of the Simplified BSD License provided in 
// LICENSE. Please retain this notice and LICENSE if you use 
// this file (or any portion of it) in your project.
// ---------------------------------------------------------

#include "depth_utils.h"
#include "ros/ros.h"
#include "marvin_convnet/DetectObjects.h"
//#include "realsense_camera/StreamSensor.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

// Marvin
#define DATATYPE 0
#include "marvin.hpp"




std::string shelf_net_arch_filename = "/home/nvidia/ctsphub-workshop-2018/04-perception/02-marvin/catkin_ws/src/marvin_convnet/models/competition/net.json";
std::string tote_net_arch_filename = "/home/nvidia/ctsphub-workshop-2018/04-perception/02-marvin/catkin_ws/src/marvin_convnet/models/competition/net.json";
std::string shelf_net_weights_filename = "/home/nvidia/ctsphub-workshop-2018/04-perception/02-marvin/catkin_ws/src/marvin_convnet/models/competition/weights_shelf.marvin";
std::string tote_net_weights_filename = "/home/nvidia/ctsphub-workshop-2018/04-perception/02-marvin/catkin_ws/src/marvin_convnet/models/competition/weights_tote.marvin";

// Service modes and names
//std::string service_name;
std::string rgb_topic_name = "/camera/rgb/image_raw";

// Directory to read/write all RGB-D files and response maps
//std::string read_directory;

// Global buffers for sensor data retrieval
int frame_width = 640;
int frame_height = 480;
uint8_t * color_buffer = new uint8_t[frame_width * frame_height * 3];


// Load Marvin FCN network architectures
marvin::Net tote_net(tote_net_arch_filename);

// Marvin responses
StorageT* color_data_CPU = NULL;
StorageT* prob_CPU_StorageT = NULL;
ComputeT* prob_CPU_ComputeT = NULL;

ros::ServiceClient client_sensor;

const int num_apc_objects = 39;

std::string shelf_bin_ids = "ABCDEFGHIJKL";




void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

ros::NodeHandle nh_p;
ros::Publisher pub = nh_p.advertise<sensor_msgs::Image>("/mask_prediction", 1);


ROS_INFO("Recieved IMAGE topic.");
cv::Mat color_frame = cv_bridge::toCvShare(msg, "bgr8")->image;
color_buffer = color_frame.data;



 // Color: BGR format, mean subtracted
  for (int r = 0; r < frame_height; ++r)
    for (int c = 0; c < frame_width; ++c) {
      color_data_CPU[0 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[0 + 3 * (c + frame_width * r)]) - ComputeT(102.9801f)); // B
      color_data_CPU[1 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[1 + 3 * (c + frame_width * r)]) - ComputeT(115.9465f)); // G
      color_data_CPU[2 * frame_height * frame_width + r * frame_width + c] = CPUCompute2StorageT(ComputeT(color_buffer[2 + 3 * (c + frame_width * r)]) - ComputeT(122.7717f)); // R
    } 




  // Run forward pass through marvin FCN
  //ROS_INFO("Forward Marvin to get segmentation results.");
  marvin::Response * rDataRGB;
  marvin::Response * rProb;
  rDataRGB = tote_net.getResponse("data_RGB");
  rProb = tote_net.getResponse("prob");



  cudaMemcpy(rDataRGB->dataGPU, color_data_CPU, rDataRGB->numBytes(), cudaMemcpyHostToDevice);

  tote_net.forward();



  cudaMemcpy(prob_CPU_StorageT, rProb->dataGPU, rProb->numBytes(), cudaMemcpyDeviceToHost);
  for (int i = 0; i < frame_height * frame_width * (num_apc_objects + 1); ++i)
    prob_CPU_ComputeT[i] = CPUStorage2ComputeT(prob_CPU_StorageT[i]);

  // Get full object list
  std::vector<std::string> all_object_names = {"background", "barkely_hide_bones", "cherokee_easy_tee_shirt", "clorox_utility_brush", "cloud_b_plush_bear", "command_hooks", "cool_shot_glue_sticks", "crayola_24_ct", "creativity_chenille_stems", "dasani_water_bottle",
                                               "dove_beauty_bar", "dr_browns_bottle_brush", "easter_turtle_sippy_cup", "elmers_washable_no_run_school_glue", "expo_dry_erase_board_eraser", "fiskars_scissors_red", "fitness_gear_3lb_dumbbell", "folgers_classic_roast_coffee", "hanes_tube_socks", "i_am_a_bunny_book",
                                               "jane_eyre_dvd", "kleenex_paper_towels", "kleenex_tissue_box", "kyjen_squeakin_eggs_plush_puppies", "laugh_out_loud_joke_book", "oral_b_toothbrush_green", "oral_b_toothbrush_red", "peva_shower_curtain_liner", "platinum_pets_dog_bowl", "rawlings_baseball",
                                               "rolodex_jumbo_pencil_cup", "safety_first_outlet_plugs", "scotch_bubble_mailer", "scotch_duct_tape", "soft_white_lightbulb", "staples_index_cards", "ticonderoga_12_pencils", "up_glucose_bottle", "womens_knit_gloves", "woods_extension_cord"};

  std::vector<std::string> selected_object_names = {"dove_beauty_bar"};//{"kleenex_paper_towels", "kleenex_tissue_box","crayola_24_ct"};
  
  unsigned short dove_color[3] = {255,255,255};
  //unsigned short viva_color[3] = {0,0,128}; //  viva = 0 0 128
  //unsigned short kleenex_color[3] = {128,128,0}; //  kleenex = 128 128 0 
  //unsigned short crayola_color[3] = {0,128,0}; // crayloa = 0 128 0

  // Remove duplicates in selected object list

  // Loop through each object in selected list
  for (int selected_idx = 0; selected_idx < 1; selected_idx++) {   //selected_object_names.size()
    std::string curr_object_name = selected_object_names[selected_idx];
    int curr_object_idx = std::distance(all_object_names.begin(), find(all_object_names.begin(), all_object_names.end(), curr_object_name));
    std::vector<ComputeT> predMap_object(prob_CPU_ComputeT + curr_object_idx * frame_height * frame_width, prob_CPU_ComputeT + (curr_object_idx + 1) * frame_height * frame_width);

/*
    curr_object_name = selected_object_names[selected_idx+1];
    curr_object_idx = std::distance(all_object_names.begin(), find(all_object_names.begin(), all_object_names.end(), curr_object_name));
    std::cout << curr_object_idx << " , "<< curr_object_name << std::endl;
    std::vector<ComputeT> predMap_object_1(prob_CPU_ComputeT + curr_object_idx * frame_height * frame_width, prob_CPU_ComputeT + (curr_object_idx + 1) * frame_height * frame_width);
*/
    /*
    curr_object_name = selected_object_names[selected_idx+2];
    curr_object_idx = std::distance(all_object_names.begin(), find(all_object_names.begin(), all_object_names.end(), curr_object_name));
    std::cout << curr_object_idx << " , "<< curr_object_name << std::endl;
    std::vector<ComputeT> predMap_object_2(prob_CPU_ComputeT + curr_object_idx * frame_height * frame_width, prob_CPU_ComputeT + (curr_object_idx + 1) * frame_height * frame_width);
*/


    cv::Mat result_mat(480, 640, CV_8UC3);

    for (size_t y = 0; y < frame_height; y++)
      for (size_t x = 0; x < frame_width; x++) {
     	       
		ComputeT p_0 = (predMap_object[y * frame_width + x]);

		//ComputeT p_1 = (predMap_object_1[y * frame_width + x]);
		//ComputeT p_2 = (predMap_object_2[y * frame_width + x]);

		int max_class = 0;
		ComputeT max_value = p_0;
		ComputeT max_value_R = 0;
		ComputeT max_value_G = 0;
		ComputeT max_value_B = 0;

    if (max_value <=0.01)
      max_value = 0;

    if (max_value >0.01)
      max_value = 1;
		max_value_R = max_value*dove_color[0];
		max_value_G = max_value*dove_color[1];
		max_value_B = max_value*dove_color[2];


		result_mat.at<cv::Vec3b>(y, x)[0] = (unsigned short)max_value_R;
		result_mat.at<cv::Vec3b>(y, x)[1] = (unsigned short)max_value_G;
		result_mat.at<cv::Vec3b>(y, x)[2] = (unsigned short)max_value_B;
     }

    cv_bridge::CvImage cv_image;
    cv::Mat result_mat_final(480, 640, CV_8UC3);
    result_mat_final = result_mat;

    cv_image.image = result_mat_final;
    cv_image.encoding = "rgb8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    pub.publish(ros_image);

}
}


int main(int argc, char **argv) {

  // Setup ROS
  ros::init(argc, argv, "marvin_convnet", ros::init_options::AnonymousName);
  ros::NodeHandle nh;


  ros::NodeHandle nh_p;


  // Setup Marvin
  ROS_INFO("Loading Marvin.");

  tote_net.Malloc(marvin::Testing);

  tote_net.loadWeights(tote_net_weights_filename);
  color_data_CPU = new StorageT[frame_width * frame_height * 3];

  prob_CPU_StorageT = new StorageT[frame_width * frame_height * (num_apc_objects + 1)];
  prob_CPU_ComputeT = new ComputeT[frame_height * frame_width * (num_apc_objects + 1)];

  ROS_INFO("Marvin Ready.");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(rgb_topic_name, 1, imageCallback);
  ROS_INFO("Image topic ready to recieve.");
  ros::spin();

  return 0;
}

