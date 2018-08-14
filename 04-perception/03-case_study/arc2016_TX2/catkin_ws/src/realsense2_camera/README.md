
After catkin_make, roslaunch realsense2_camera rs_rgbd.launch


(Optional)
catkin_make --pkg realsense2_camera -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
