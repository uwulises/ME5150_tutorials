import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

class KUKAPoseViewer:

    def __init__(self, values_list=[0, -90, 90, 0, 0, 0]):
        self.values_list = values_list
        self.poses = np.array([])
        self.ax3 = None

    def get_pose_to_origin_pose_kuka(self):

        origin = np.identity(4)
        Z1 = np.linalg.multi_dot([self.translation_z(675), self.rot_z(np.deg2rad(self.values_list[0]))])
        X1 = np.linalg.multi_dot([self.translation_x(300), self.rot_x(np.deg2rad(-90))])
        Z2 = np.linalg.multi_dot([self.translation_z(0), self.rot_z(np.deg2rad(self.values_list[1]))])
        X2 = np.linalg.multi_dot([self.translation_x(650), self.rot_x(np.deg2rad(0))])
        Z3 = np.linalg.multi_dot([self.translation_z(0), self.rot_z(np.deg2rad(self.values_list[2]-90))])
        X3 = np.linalg.multi_dot([self.translation_x(155), self.rot_x(np.deg2rad(-90))])
        Z4 = np.linalg.multi_dot([self.translation_z(600), self.rot_z(np.deg2rad(self.values_list[3]))])
        X4 = np.linalg.multi_dot([self.translation_x(0), self.rot_x(np.deg2rad(90))])
        Z5 = np.linalg.multi_dot([self.translation_z(0), self.rot_z(np.deg2rad(self.values_list[4]))])
        X5 = np.linalg.multi_dot([self.translation_x(0), self.rot_x(np.deg2rad(-90))])
        Z6 = np.linalg.multi_dot([self.translation_z(125), self.rot_z(np.deg2rad(self.values_list[5]))])
        X6 = np.linalg.multi_dot([self.translation_x(0), self.rot_x(np.deg2rad(0))])
        A1 = np.linalg.multi_dot([origin,Z1])
        A2 = np.linalg.multi_dot([A1, X1, Z2])
        A3 = np.linalg.multi_dot([A2, X2, Z3])
        A4 = np.linalg.multi_dot([A3, X3, Z4])
        A5 = np.linalg.multi_dot([A4, X4, Z5])
        A6 = np.linalg.multi_dot([A5, X5, Z6, X6])

        poses = np.array([origin, A1, A2, A3, A4, A5, A6])
        self.poses = poses
        return self.poses
    
    def translation_x(self,x):

        return np.array([[1, 0, 0, x], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    def translation_y(self,y):

        return np.array([[1, 0, 0, 0], [0, 1, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    def translation_z(self, z):

        return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, z], [0, 0, 0, 1]])

    def rot_x(self, qx):

        return np.array([[1, 0, 0, 0], [0, np.cos(qx), -np.sin(qx), 0], [0, np.sin(qx), np.cos(qx), 0], [0, 0, 0, 1]])

    def rot_y(self, qy):

        return np.array([[np.cos(qy), 0, np.sin(qy), 0], [0, 1, 0, 0], [-np.sin(qy), 0, np.cos(qy), 0], [0, 0, 0, 1]])

    def rot_z(self, qz):

        return np.array([[np.cos(qz), -np.sin(qz), 0, 0], [np.sin(qz), np.cos(qz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def draw_axes_tf(self, poses, name="", color="k"):
        
        for pose in poses:
            self.ax3.set_xlim(0, 1500)
            self.ax3.set_ylim(-150, 1500)
            self.ax3.set_zlim(0, 2000)
            self.ax3.set_title("KUKA")
            self.ax3.set_xlabel('x-axis')
            self.ax3.set_ylabel('y-axis')
            self.ax3.set_zlabel('z-axis')
            self.ax3.scatter(xs=[0], ys=[0], zs=[0], marker='o', color=color)
            origin_pose = np.transpose(pose)[3, 0:3]
            x_rot = np.linalg.multi_dot([pose, [1, 0, 0, 0]])
            y_rot = np.linalg.multi_dot([pose, [0, 1, 0, 0]])
            z_rot = np.linalg.multi_dot([pose, [0, 0, 1, 0]])
            self.ax3.quiver(origin_pose[0], origin_pose[1], origin_pose[2], x_rot[0],
                    x_rot[1], x_rot[2], length=100, normalize=True, color='r')
            self.ax3.quiver(origin_pose[0], origin_pose[1], origin_pose[2], y_rot[0],
                    y_rot[1], y_rot[2], length=100, normalize=True, color='g')
            self.ax3.quiver(origin_pose[0], origin_pose[1], origin_pose[2], z_rot[0],
                    z_rot[1], z_rot[2], length=100, normalize=True, color='b')
            self.ax3.scatter(xs=[origin_pose[0]], ys=[origin_pose[1]],
                    zs=[origin_pose[2]], marker='o')

    def slider(self):
        fig = plt.figure(figsize=(6, 8))
        self.ax3 = fig.add_subplot(111, projection='3d', position=[0.1, 0.3, 0.8, 0.8])
        self.get_pose_to_origin_pose_kuka()
        self.draw_axes_tf(self.poses)
        # Create sliders
        self.slider1_ax = plt.axes([0.2, 0.1, 0.65, 0.03])
        self.slider2_ax = plt.axes([0.2, 0.15, 0.65, 0.03])
        self.slider3_ax = plt.axes([0.2, 0.2, 0.65, 0.03])
        self.slider4_ax = plt.axes([0.2, 0.25, 0.65, 0.03])
        self.slider5_ax = plt.axes([0.2, 0.3, 0.65, 0.03])
        self.slider6_ax = plt.axes([0.2, 0.35, 0.65, 0.03])
        resetax = plt.axes([0.8, 0.025, 0.1, 0.04])

        self.slider1 = Slider(self.slider1_ax, 'A1', -185, 185, valinit=0)
        self.slider2 = Slider(self.slider2_ax, 'A2', -100, 115, valinit=-90)
        self.slider3 = Slider(self.slider3_ax, 'A3', -210, 100, valinit=90)
        self.slider4 = Slider(self.slider4_ax, 'A4', -350, 350, valinit=0)
        self.slider5 = Slider(self.slider5_ax, 'A5', -130, 130, valinit=0)
        self.slider6 = Slider(self.slider6_ax, 'A6', -350, 350, valinit=0)
        #add a button for reset
        
        button = plt.Button(resetax, 'Reset', color='white', hovercolor='0.975')
        button.on_clicked(self.reset)
        self.slider1.on_changed(self.update)
        self.slider2.on_changed(self.update)
        self.slider3.on_changed(self.update)
        self.slider4.on_changed(self.update)
        self.slider5.on_changed(self.update)
        self.slider6.on_changed(self.update)
        plt.show()
        
        

    def update(self,val):
        #clean the last plot
        self.ax3.cla()
        slider_values = np.array([self.slider1.val, self.slider2.val, self.slider3.val, self.slider4.val, self.slider5.val, self.slider6.val])
        self.values_list = slider_values
        self.get_pose_to_origin_pose_kuka()
        self.draw_axes_tf(self.poses)
        pass

    #create a reset for sliders
    def reset(self, event):
        self.slider1.reset()
        self.slider2.reset()
        self.slider3.reset()
        self.slider4.reset()
        self.slider5.reset()
        self.slider6.reset()
        pass



#test
if __name__ == "__main__":
     
     kuka = KUKAPoseViewer()
     kuka.slider()
