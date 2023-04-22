import pybullet as p
import pybullet_data
import time
import numpy as np
from pca_opencv import pca_algorithm
import cv2



def get_img_rgba(width=240, height=240, fov=60, near=0.02, far=2, camposition=[1.3, 0, 1.5],distance=0.1,yaw=0,pitch=-90,roll=0):
    aspect = width / height
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition,distance,yaw,pitch,roll,upAxisIndex=2)
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    w_img, h_img, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    depth_buffer_opengl = np.reshape(depthImg, [width, height])
    depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)
    return rgbaImg

def detect_borders(img, low=20, high=200):

    new_img = np.copy(img)
    edges = cv2.Canny(new_img, low, high, None,3)
    return edges

def pose_object(img):
    new_img = np.copy(img)
    return pca_algorithm(new_img)

