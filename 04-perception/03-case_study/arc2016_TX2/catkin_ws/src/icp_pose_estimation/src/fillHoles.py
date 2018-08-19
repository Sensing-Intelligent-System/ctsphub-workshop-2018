#!/usr/bin/env python
import numpy as np
import cv2  
from loadscene import Scene

def fillHoles(depth_frame):
	depth = np.array(depth_frame)
	image_shape = np.shape(depth_frame)
	# print image_shape
	roiDepth = np.array(depth_frame[1:(image_shape[0]-1),1:(image_shape[1]-1)])
	# Horizontal averaging
	rightDepth = np.array(depth_frame[1:(image_shape[0]-1),2:(image_shape[1])])
	leftDepth = np.array(depth_frame[1:(image_shape[0]-1),0:(image_shape[1]-2)])
	horzIdx = np.multiply((roiDepth == 0),(leftDepth > 0)) 
	horzIdx =np.multiply((rightDepth > 0),horzIdx)
	interp = ((leftDepth+rightDepth)/2)
	roiDepth[horzIdx] = roiDepth[horzIdx] + interp[horzIdx]


	# Vertical averaging
	upperDepth = np.array(depth_frame[0:(image_shape[0]-2),1:(image_shape[1]-1)])
	lowerDepth = np.array(depth_frame[2:(image_shape[0]),1:(image_shape[1]-1)])
	vertIdx = np.multiply((roiDepth == 0),(upperDepth > 0)) 
	vertIdx =np.multiply((lowerDepth > 0),vertIdx)
	interp = ((upperDepth+lowerDepth)/2)
	roiDepth[vertIdx] = roiDepth[vertIdx] + interp[vertIdx]
	# print sum(sum(abs(roiDepth)))

	#Diagonal averaging 1 (upper left to lower right)
	upperLeftDepth = np.array(depth_frame[0:(image_shape[0]-2),0:(image_shape[1]-2)])
	lowerRightDepth = np.array(depth_frame[2:(image_shape[0]),2:(image_shape[1])])
	diag1Idx = np.multiply((roiDepth == 0),(upperLeftDepth > 0)) 
	diag1Idx = np.multiply(diag1Idx,(lowerRightDepth > 0)) 
	interp = ((upperLeftDepth+lowerRightDepth)/2)
	roiDepth[diag1Idx] = roiDepth[diag1Idx] + interp[diag1Idx]
	# print sum(sum(abs(roiDepth)))

	#Diagonal averaging 2 (lower left to upper right)
	lowerLeftDepth = np.array(depth_frame[2:(image_shape[0]),0:(image_shape[1]-2)])
	upperRightDepth = np.array(depth_frame[0:(image_shape[0]-2),2:(image_shape[1])])
	diag2Idx = np.multiply((roiDepth == 0),(lowerLeftDepth > 0)) 
	diag2Idx = np.multiply((upperRightDepth > 0),diag2Idx)
	interp = ((lowerLeftDepth+upperRightDepth)/2)
	roiDepth[diag2Idx] = roiDepth[diag2Idx] + interp[diag2Idx]
	# print sum(sum(abs(roiDepth)))

	denom = np.ones(np.shape(roiDepth));
	# zeros_idx = horzIdx
	# zeros_idx = np.multiply(zeros_idx,vertIdx )
	# zeros_idx = np.multiply(zeros_idx,diag1Idx )
	# zeros_idx = np.multiply(zeros_idx,diag2Idx)
	denom[horzIdx] = 0
	denom[vertIdx] = 0
	denom[diag1Idx] = 0
	denom[diag2Idx] = 0
	denom[horzIdx] = denom[horzIdx]+1
	denom[vertIdx] = denom[vertIdx]+1
	denom[diag1Idx] = denom[diag1Idx]+1
	denom[diag2Idx] = denom[diag2Idx]+1
	roiDepth = roiDepth/denom
	depth[1:image_shape[0]-1,1:image_shape[1]-1] = np.array(roiDepth)

	#Vertical averaging on edges

	roiEdges = np.vstack( (depth[1:(image_shape[0]-1),0] , depth[1:image_shape[0]-1,image_shape[1]-1]) )
	upperDepth = np.vstack((depth[0:(image_shape[0]-2),0],depth[0:(image_shape[0]-2),image_shape[1]-1]))
	lowerDepth = np.vstack((depth[2:(image_shape[0]-0),0],depth[2:(image_shape[0]-0),image_shape[1]-1]))


	vertIdx = np.multiply((roiEdges == 0),(upperDepth > 0)) 
	vertIdx = np.multiply((lowerDepth > 0),vertIdx)
	interp = (lowerDepth+upperDepth)/2
	roiEdges[vertIdx] = interp[vertIdx]
	depth[1:(image_shape[0]-1),0] = np.array(roiEdges[0,:])
	depth[1:(image_shape[0]-1),image_shape[1]-1] = np.array(roiEdges[1,:])

	#Horizontal averaging on edges
	roiEdges = np.vstack((depth[0,1:image_shape[1]-2],depth[image_shape[0]-1,1:image_shape[1]-2] ))
	rightDepth = np.vstack((depth[0,2:image_shape[1]-1],depth[image_shape[0]-1,2:image_shape[1]-1] ))
	leftDepth = np.vstack((depth[0,0:image_shape[1]-3],depth[image_shape[0]-1,0:image_shape[1]-3] ))
	horzIdx = np.multiply((roiEdges == 0),(rightDepth > 0)) 
	horzIdx = np.multiply((leftDepth > 0),horzIdx)
	interp = (rightDepth +leftDepth)/2
	roiEdges[horzIdx] = interp[horzIdx]
	depth[0,1:(image_shape[1]-2)] = np.array(roiEdges[0,:])
	depth[image_shape[0]-1,1:(image_shape[1]-2)]= np.array(roiEdges[1,:])

	return depth
