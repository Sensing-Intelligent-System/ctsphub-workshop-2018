#!/usr/bin/env python

# numpy
import numpy as np

# OpenCV
import cv2
import copy

def depth_remove_nan(cv_depth):
    cv_depth[np.isnan(cv_depth)] = 0
    return cv_depth

'''
def depth_fill_holes(cv_depth):
    depht_shape = cv_depth.shape
    proposal_pixels = np.where(cv_depth == 0)
    mask_size = 3
    
    assert len(proposal_pixels[0]) == len(proposal_pixels[1])
    for i in range(0, len(proposal_pixels[0])):
        y = proposal_pixels[0][i]
        x = proposal_pixels[1][i]
        #print y,x



        try:
            local_area = cv_depth[y-mask_size/2:y+mask_size/2,x-mask_size/2:x+mask_size/2]
            valid_count = len(local_area!=0)

            if valid_count > 0:
                cv_depth[y,x] = np.sum(local_area)/valid_count

        except:
            cv_depth[y,x] = 0
'''



def image_in_workrange(cv_RGB, cv_depth):
    sr300_depth_range = np.array([0.3,2])  # sr300 depth is working in 0.3m ~ 2m
    cv_depth = depth_remove_nan(cv_depth)
    invalid_pixs = np.logical_not(np.logical_and(cv_depth >= sr300_depth_range[0], cv_depth <= sr300_depth_range[1]))
    valid_RGB = copy.copy(cv_RGB)
    valid_RGB[invalid_pixs, :] = [0,0,0]
    return valid_RGB


def search_desired_pixel(cv_RGB, cv_depth):
    search_depth_range = np.array([0.3, 0.5])	# desired range 0.3m ~ 0.5m
    invalid_pixs = np.logical_not(np.logical_and(cv_depth >= search_depth_range[0], cv_depth <= search_depth_range[1]))
    desired_RGB = copy.copy(cv_RGB)
    desired_RGB[invalid_pixs, :] = [0,0,0]
    return desired_RGB