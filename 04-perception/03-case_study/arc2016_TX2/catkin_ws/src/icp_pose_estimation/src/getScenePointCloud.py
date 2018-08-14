#!/usr/bin/env python
import numpy as np
import cv2  
from loadscene import Scene
from sensor_msgs.msg import PointCloud2, PointField
import rospy
def  getScenePointCloud(sceneData):
	if (sceneData.env)=='tote':
		viewBounds = np.array([-0.3, 0.3],[ -0.4, 0.4],[ -0.05, 0.2])
	else:
		viewBounds = np.array([-0.01, 0.4, -0.3, 0.3, -0.06, 0.20])
		# viewBounds = np.array([-0.01, 0.4],[-0.17, 0.17],[ -0.06, 0.2])
	numFrames = len(sceneData.colorFrames)
	for i in range(0,numFrames):
		colorFrame = np.array(sceneData.colorFrames[i])
		depthFrame = np.array(sceneData.depthFrames[i])
		extCam2World = np.array(sceneData.extCam2World[i])
		extCam2Bin = np.array(np.dot(sceneData.world_to_bin_transformation_matrix,extCam2World))
		#Project depth into camera space
		x = np.arange(0, 640)
		y = np.arange(0, 480)
		[pixX,pixY] = np.meshgrid( x,y)
		camX = np.multiply((pixX-sceneData.color_camera_intrinsic_matrix[0,2]),depthFrame)/sceneData.color_camera_intrinsic_matrix[0,0]
		camY = np.multiply((pixY-sceneData.color_camera_intrinsic_matrix[1,2]),depthFrame)/sceneData.color_camera_intrinsic_matrix[1,1]
		camZ = depthFrame
		validDepth = (camZ > 0)
		camPts = np.vstack(( camX[validDepth],camY[validDepth],camZ[validDepth] ))
		# print (max(camPts[0,:]))
		# print (max(camPts[1,:]))
		# print (max(camPts[2,:]))
		reg = np.shape(camPts)
		translation = np.tile((extCam2Bin[0:3,3]),(reg[1],1))
		camPts = np.dot(extCam2Bin[0:3,0:3],camPts) + translation.T
		camPts = camPts.T
		# print max(camPts[:,0])
		# print max(camPts[:,1])
		# print max(camPts[:,2])
		# print max(camPts[:,3])
		# print max(camPts[:,4])
		# print max(camPts[:,5])
		# validpoint = (camPts[:,0]>viewBounds[0])
		# camPts = camPts[validpoint,:]
		# print camPts
		validpoint = (camPts[:,0]<viewBounds[1]) * (camPts[:,0]>viewBounds[0]) * (camPts[:,1]<viewBounds[3]) * (camPts[:,1]>viewBounds[2]) * (camPts[:,2]<viewBounds[5]) * (camPts[:,2]>viewBounds[4])
		camPts = camPts[validpoint,:]

		#  * 
		# camPts = camPts[validDepth,:]

		# Get vertex colors



		colorR = colorFrame[:,:,2]
		colorG = colorFrame[:,:,1]
		colorB = colorFrame[:,:,0]
		# colorPts = np.left_shift(np.uint8(colorR), 16) + np.left_shift(np.uint8(colorG), 8) + np.uint8(colorB)
		# colorPts = colorPts[validDepth]
		# colorPts = np.float32((colorPts.T))

		# colorPts = colorPts[validpoint]
		# print np.shape(colorPts)
		# print np.shape(camPts)
		# pointarray  = np.zeros((np.shape(colorPts)[0],4), dtype=np.float32)
		# pointarray[:,0:3] = camPts
		# pointarray[:,3] = colorPts
		# print np.shape(pointarray)
		colorPts = np.vstack((colorR[validDepth],colorG[validDepth],colorB[validDepth]))
		colorPts = np.float32((colorPts.T)/255.0)
		colorPts = colorPts[validpoint,:]
		# print np.shape(camPts)
		# print np.shape(colorPts)
		pointarray = np.hstack((camPts,colorPts))
		print np.shape(pointarray )
		timestamp = rospy.get_rostime()
		sceneData.points.append(array_to_pointcloud2(np.float32(pointarray), stamp=timestamp, frame_id="/camera"))
	return 
		# array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None, merge_rgb=False):


def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = 1
    # print cloud_msg.height
    #Product Field
    cloud_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        # PointField('rgb', 12, PointField.FLOAT32, 1)]
		PointField('r', 12, PointField.FLOAT32, 1),
		PointField('g', 16, PointField.FLOAT32, 1),
		PointField('b', 20, PointField.FLOAT32, 1)]
    
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = 24
    cloud_msg.row_step = cloud_msg.point_step*cloud_msg.height
    cloud_msg.is_dense = True
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg 