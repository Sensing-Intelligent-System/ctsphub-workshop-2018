# arc2016_TX2
Used for Central Taiwan Science Park workshop

#Pre-requiste
A Jetson TX-2 which has realsense SR300 kernel

#how to run
	Compile code
	$cd arc2016_TX2/catkin_ws
	$catkin_make

	Build tote background cloud (optional) (The default path is "/home/nvidia/arc2016_TX2/catkin_ws/src/icp_pose_estimation/src/model/bins")
	$rosrun model_generating cloud_save 
	
		
	Configure model path (optional)
	The demo read the model from "/home/nvidia/arc2016_TX2/catkin_ws/src/icp_pose_estimation/src/model"
	If you need to reconfigure the model path, please revise line 31, 32 in icp_pose_estimation.cpp which is in icp_pose_estimation package
		

	Run demo (source devet/setup.sh before you run the launch or node)
	$roslaunch realsense2_camera camera_rs_rgbd.launch
	$rosrun marvin_convnet detect
	$rosrun icp_pose_estimation demo
	$rviz 
	Then you can see the result in RVIZ
	
