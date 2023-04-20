import pybullet as p
import pybullet_data
import time
import numpy as np
from pca_opencv import pca_algorithm
import cv2
width = 240
height = 240
fov = 60
aspect = width / height
near = 0.02
far = 2
view_matrix = p.computeViewMatrixFromYawPitchRoll([1.3, 0, 1.5],distance=0.1,yaw=0,pitch=-90,roll=0,upAxisIndex=2)
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

def get_img_rgba():
    w_img, h_img, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,  flags= p.ER_NO_SEGMENTATION_MASK)
    depth_buffer_opengl = np.reshape(depthImg, [width, height])
    depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)
    return rgbaImg


def pose_object(img):
    return pca_algorithm(img)

