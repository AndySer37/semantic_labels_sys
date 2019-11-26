import numpy as np
import argparse
import cv2


def rotate_cv(image, center=None, scale=1.0):

    (h, w) = image.shape[:2]
    if center is None:
        center = (w / 2, h / 2)
 
    M_90 = cv2.getRotationMatrix2D(center, 90, scale)
    M_180 = cv2.getRotationMatrix2D(center, 180, scale)
    M_270 = cv2.getRotationMatrix2D(center, 270, scale)
    M_90[0,2] = 0
    M_90[1,2] = w
    M_270[0,2] = h
    M_270[1,2] = 0
    rotated_90 = cv2.warpAffine(image, M_90, (h, w))
    rotated_180 = cv2.warpAffine(image, M_180, (w, h))
    rotated_270 = cv2.warpAffine(image, M_270, (h, w))
 
    return [image, rotated_90, rotated_180, rotated_270]




def rotate_back(image, center=None, scale=1.0, angle=0):

    (h, w) = image.shape[:2]
 
    if center is None:
        center = (w / 2, h / 2)
 
    M = cv2.getRotationMatrix2D(center, angle, scale)


    rotated = cv2.warpAffine(image, M, (w, h))

 
    return rotated



def rotate_back_change_h_w(image, center=None, scale=1.0, angle=0):

    (h, w) = image.shape[:2]
 
    if center is None:
        center = (h / 2, w / 2)
 
    M = cv2.getRotationMatrix2D(center, angle, scale)
    if angle == -90:
        M[0,2] = h
        M[1,2] = 0
    elif angle == -270:
        M[0,2] = 0
        M[1,2] = w       
    rotated = cv2.warpAffine(image, M, (h, w))

    return rotated
