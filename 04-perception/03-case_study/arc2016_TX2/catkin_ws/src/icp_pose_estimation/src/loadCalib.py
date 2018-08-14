#!/usr/bin/env python
import numpy as np
import cv2  
from loadscene import Scene

def loadCalib(calibdir,sceneData):
	if (sceneData.env) == 'tote':
		baseFrameIdx = 1
		sceneData.calibPoseFile = calibdir + '/tote/cam.poses.txt'
	else :
		baseFrameIdx = 8
		sceneData.calibPoseFile = calibdir + '/shelf/' + '/cam.poses.'+sceneData.binId + '.txt';
	extPivot2World = sceneData.camera_to_world_transformation_matrix;
	extCam2Pivot_reg = []
	calibPoseInfoFileId = open(sceneData.calibPoseFile, "r")
	while True:
		try:
			extCam = np.zeros([4,4])
			calibPoseInfoFileId.readline()
			for i in range(0,4):
				matrix_row = calibPoseInfoFileId.readline()
				row_value = matrix_row.split('\t')
				for j in range(0,4):
					extCam[i,j]= float(row_value[j])		
			calibPoseInfoFileId.readline()
			extCam2Pivot_reg.append((extCam))
		except:
			break
	sceneData.extCam2Pivot = extCam2Pivot_reg
	for i in range(0,len(sceneData.colorFrames)):
		sceneData.extCam2World.append(np.dot(extPivot2World, sceneData.extCam2Pivot[i]))



