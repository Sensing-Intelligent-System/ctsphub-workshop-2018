#!/usr/bin/env python

#Author: Yung Shan Su
#Rewrite ARC 2016 MIT-Princeton method in py
import numpy as np
import cv2  
from sensor_msgs.msg import PointCloud2
import rospy
# toolboxPath = '/home/michael/apc-vision-toolbox/'
# tmpDataPath = '/home/michael/apc-vision-toolbox/toolbox/data/tmp' # Temporary directory used by marvin_convnet, where all RGB-D images and detection masks are saved
# modelsPath = '/home/michael/apc-vision-toolbox/ros-packages/catkin_ws/src/pose_estimation/src/models/objects'#Directory holding pre-scanned object models
# scenePath = toolboxPath + 'data/sample/scene-0000' # Directory holding the RGB-D data of the sample scene
# calibPath = toolboxPath + 'data/sample/calibration'# Directory holding camera pose calibration data for the sample scene


class Scene:
	def __init__(self, path):
		self.env = "shelf"
		self.binId = "B"
		self.path = path
		self.camInfoFile = path + '/cam.info.txt'
		self.camInfoFileId = open(self.camInfoFile, "r")
		self.objectlist = []
		self.color_camera_intrinsic_matrix = np.zeros([3,3])
		self.depth_camera_intrinsic_matrix = np.zeros([3,3])
		self.depth_to_color_camera_extrinsic_matrix= np.zeros([4,4])
		self.bin_to_world_transformation_matrix = np.zeros([4,4])
		self.world_to_bin_transformation_matrix = np.zeros([4,4])
		self.camera_to_world_transformation_matrix= np.zeros([4,4])
		self.depthFrames = []
		self.rawDepthFrames = []
		self.colorFrames = []
		self.extCam2Pivot = []
		self.extCam2World = []
		self.points = []

	def decode_scene(self):
		self.decode_environment()
		self.decode_color_camera_intrinsic_matrix()
		self.decode_depth_camera_intrinsic_matrix()
		self.decode_depth_to_color_camera_extrinsic_matrix()
		self.decode_bin_to_world_transformation_matrix()
		self.decode_camera_to_world_transformation_matrix()


	def decode_environment(self):
		reg = self.camInfoFileId.readline()
		env_split = reg.split('Environment: ')
		self.env = env_split[1]
		reg = self.camInfoFileId.readline()
		env_split = reg.split('Bin ID: ')
		self.binId = (env_split[1].split('\n'))[0]
		reg = self.camInfoFileId.readline()
		product_split = reg.split('"')
		i = 1
		while i < len(product_split)-1:
			self.objectlist.append(product_split[i])
			i = i+2

	def decode_color_camera_intrinsic_matrix(self):
		self.camInfoFileId.readline()
		self.camInfoFileId.readline()	
		for i in range(0,3):
			matrix_row = self.camInfoFileId.readline()
			row_value = matrix_row.split('\t')
			for j in range(0,3):
				self.color_camera_intrinsic_matrix[i,j]=(float(row_value[j]))
		# print self.color_camera_intrinsic_matrix

	def decode_depth_camera_intrinsic_matrix(self):
		self.camInfoFileId.readline()
		self.camInfoFileId.readline()
		for i in range(0,3):
			matrix_row = self.camInfoFileId.readline()
			row_value = matrix_row.split('\t')
			for j in range(0,3):
				self.depth_camera_intrinsic_matrix[i,j]=(float(row_value[j]))
		# print self.depth_camera_intrinsic_matrix

	def decode_depth_to_color_camera_extrinsic_matrix(self):
		self.camInfoFileId.readline()
		self.camInfoFileId.readline()
		for i in range(0,4):
			matrix_row = self.camInfoFileId.readline()
			row_value = matrix_row.split('\t')
			for j in range(0,4):
				self.depth_to_color_camera_extrinsic_matrix[i,j]= float(row_value[j])
		# print self.depth_to_color_camera_extrinsic_matrix


	def decode_bin_to_world_transformation_matrix(self):
		self.camInfoFileId.readline()
		self.camInfoFileId.readline()
		for i in range(0,4):
			matrix_row = self.camInfoFileId.readline()
			row_value = matrix_row.split('\t')
			for j in range(0,4):
				self.bin_to_world_transformation_matrix[i,j]= float(row_value[j])
		self.world_to_bin_transformation_matrix = np.linalg.inv(self.bin_to_world_transformation_matrix)
		# print self.bin_to_world_transformation_matrix
	def decode_camera_to_world_transformation_matrix(self):
		for i in range(0,42):
			self.camInfoFileId.readline()
		self.camInfoFileId.readline()
		self.camInfoFileId.readline()
		for i in range(0,4):
			matrix_row = self.camInfoFileId.readline()
			row_value = matrix_row.split('\t')
			for j in range(0,4):
				self.camera_to_world_transformation_matrix[i,j]= float(row_value[j])
		# print self.camera_to_world_transformation_matrix
	def read_depth_images(self):	
		for i in range (0,15):
			buf ="%s/frame-0000%02d.depth.png" % (self.path,i)
			filename = buf
			image = cv2.imread(filename,-1)
			image = (image<<13) | (image>>3)
			image = image/10000.0
			self.depthFrames.append(image)
		# print np.shape(self.depthFrames)

	def read_raw_depth_images(self):
		for i in range (0,15):
			buf ="%s/raw/frame-0000%02d.depth.png" % (self.path,i,)
			filename = buf
			image = cv2.imread(filename,cv2.IMREAD_GRAYSCALE)
			self.rawDepthFrames.append(image)
		# print np.shape(self.rawDepthFrames)

	def read_rgb_images(self):
		for i in range (0,15):
			buf ="%s/frame-0000%02d.color.png" % (self.path,i)
			filename = buf
			image = cv2.imread(filename)
			self.colorFrames.append(image)





def main():
	camera_scene = Scene(scenePath)
	camera_scene.decode_scene()
	camera_scene.read_depth_images()
	camera_scene.read_raw_depth_images()
	camera_scene.read_rgb_images()


if __name__ == '__main__': 
	main()


