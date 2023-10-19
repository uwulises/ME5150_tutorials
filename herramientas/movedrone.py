import numpy as np
import keyboard
import cv2

class MoveDrone:
    def __init__(self, pose):
        self.pose = np.array(pose) # [x, y, z, roll, pitch, yaw]
        self.vel = 15 # velocidad de movimiento
        self.target_pose = np.array(pose) # [x, y, z, roll, pitch, yaw]

    def update_pose(self):
        dif = np.array(self.target_pose) - np.array(self.pose)
        self.pose += dif * self.vel * 1/240

    def get_pose(self):
        return self.pose
    
    def set_target_pose(self, pose):
        self.target_pose = pose

    def move_by_key(self, keys):
        d = 0.001 # variación de posición
        d_degrees = 0.001 # variación de ángulo
        if keys.get(114)==1:
            # Go up, R
            self.target_pose[2] += d
        elif keys.get(102)==1:
            # Go down, F
            self.target_pose[2] -= d
        elif keys.get(119)==1:
            # Go forward, W
            self.target_pose[0] += d * np.cos(self.pose[5])
            self.target_pose[1] += d * np.sin(self.pose[5])
        elif keys.get(115)==1:
            # Go back, S
            self.target_pose[0] -= d * np.cos(self.pose[5])
            self.target_pose[1] -= d * np.sin(self.pose[5])
        elif keys.get(97)==1:
            # Go left, A
            self.target_pose[1] += d * np.cos(self.pose[5])
            self.target_pose[0] -= d * np.sin(self.pose[5])
        elif keys.get(100)==1:
            # Go right, D
            self.target_pose[1] -= d * np.cos(self.pose[5])
            self.target_pose[0] += d * np.sin(self.pose[5])
        elif keys.get(113)==1:
            # Go counterclockwise, Q
            self.target_pose[5] += d_degrees
        elif keys.get(101)==1:
            # Go clockwise, E
            self.target_pose[5] -= d_degrees
