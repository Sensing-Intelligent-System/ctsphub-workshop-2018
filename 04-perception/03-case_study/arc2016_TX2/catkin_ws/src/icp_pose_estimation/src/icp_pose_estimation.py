#!/usr/bin/env python
import numpy as np
import cv2  
import rospy
from loadscene import Scene
from loadCalib import *
from fillHoles import *
from getScenePointCloud import *
from sensor_msgs.msg import PointCloud2


toolboxPath = '/home/michael/apc-vision-toolbox/'
tmpDataPath = '/home/michael/apc-vision-toolbox/toolbox/data/tmp' # Temporary directory used by marvin_convnet, where all RGB-D images and detection masks are saved
modelsPath = '/home/michael/apc-vision-toolbox/ros-packages/catkin_ws/src/pose_estimation/src/models/objects'#Directory holding pre-scanned object models
scenePath = toolboxPath + 'data/sample/scene-0000' # Directory holding the RGB-D data of the sample scene
calibPath = toolboxPath + 'data/sample/calibration'# Directory holding camera pose calibration data for the sample scene
point_pub = rospy.Publisher("scene_pointcloud", PointCloud2, queue_size=10)


def scene_pointcloud_publish(point):
	point_pub.publish(point)
	r = rospy.Rate(10)
	r.sleep()
	return

def main():
	rospy.init_node('product_point_segment', anonymous=True)
	camera_scene = Scene(scenePath)
	camera_scene.decode_scene()
	camera_scene.read_depth_images()
	camera_scene.read_raw_depth_images()
	camera_scene.read_rgb_images()
	print 'Finish Loading scene'
	loadCalib(calibPath,camera_scene)
	print 'Finish Loading Camera extrinsic matrix'
	# print np.shape(camera_scene.depthFrames)
	for i in range(0,len(camera_scene.depthFrames)):
		camera_scene.depthFrames[i] = fillHoles(camera_scene.depthFrames[i]);
	#Load scene point cloud
	getScenePointCloud(camera_scene)
	print camera_scene.points[7].height
	print camera_scene.points[7].width
	while True:
		scene_pointcloud_publish(camera_scene.points[7])
			
	

	




if __name__ == '__main__': 
	main()