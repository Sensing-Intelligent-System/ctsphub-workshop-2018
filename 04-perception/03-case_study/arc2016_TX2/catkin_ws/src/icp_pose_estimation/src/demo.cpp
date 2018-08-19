#include "icp_pose_estimation.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "cloud_icp_demo");
    Icp_pose_estimation icp_pose_estimation;
    ros::spin();
    return 0;
}