import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

class SCARAPoseViewer:

    def __init__(self, val_list=[0,0,0,0]):
        #values list := [hombro, codo, z, a], in degrees except z[mm]
        self.values_list=val_list
        self.poses = np.array([])
        self.ax3 = None

    def get_pose_to_origin_pose_scara(self):
        # usage np.linalg.multi_dot([Z1, X1, ...., Zn, Xn]), for rotations: rot_axis(np.deg2rad(self.values_list[i]))
        origen = np.identity(4)
        #Primer link, del suelo al primer eje
        Z1 = np.identity(4)
        X1 = np.identity(4)
        #Segundo link, del primer eje al segundo eje
        Z2 = np.identity(4)
        X2 = np.identity(4)
        #Tercer link, del segundo eje al tercer eje (Z)
        Z3 = np.identity(4)
        X3 = np.identity(4)
        #Cuarto link, del tercer eje al cuarto eje (A)
        Z4 = np.identity(4)
        X4 = np.identity(4)
        #END EFFECTOR
        GC = np.identity(4)
        #poses
 

        poses = np.array([origen])

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
            self.ax3.set_xlim(-700, 700)
            self.ax3.set_ylim(-700, 700)
            self.ax3.set_zlim(0, 600)
            self.ax3.set_xlabel('x-axis')
            self.ax3.set_ylabel('y-axis')
            self.ax3.set_zlabel('z-axis')
            self.ax3.scatter(xs=[0], ys=[0], zs=[0], marker='o', color=color)
            origin_pose = np.transpose(pose)[3, 0:3]
            # multiply the first 3 rows of pose matrix by 1000 for mm
            pose[0:3, 3] = pose[0:3, 3]*1000
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
        # Create sliders
        self.slider1_ax = plt.axes([0.2, 0.1, 0.65, 0.03])
        self.slider2_ax = plt.axes([0.2, 0.15, 0.65, 0.03])
        self.slider3_ax = plt.axes([0.2, 0.2, 0.65, 0.03])
        self.slider4_ax = plt.axes([0.2, 0.25, 0.65, 0.03])
        resetax = plt.axes([0.8, 0.025, 0.1, 0.04])

        self.slider1 = Slider(self.slider1_ax, 'Hombro', 0, 180, valinit=0)
        self.slider2 = Slider(self.slider2_ax, 'Codo', -180, 180, valinit=0)
        self.slider3 = Slider(self.slider3_ax, 'Z [mm]', 0, 180, valinit=180)
        self.slider4 = Slider(self.slider4_ax, 'A', -180, 180, valinit=0)
        #add a button for reset
        
        button = plt.Button(resetax, 'Reset', color='white', hovercolor='0.975')
        button.on_clicked(self.reset)
        self.slider1.on_changed(self.update)
        self.slider2.on_changed(self.update)
        self.slider3.on_changed(self.update)
        self.slider4.on_changed(self.update)
        plt.show()
        
        

    def update(self,val):
        #clean the last plot
        self.ax3.cla()
        slider_values = np.array([self.slider1.val, self.slider2.val, self.slider3.val, self.slider4.val])
        self.values_list = slider_values
        self.get_pose_to_origin_pose_scara()
        self.draw_axes_tf(self.poses)
        pass

    #create a reset for sliders
    def reset(self, event):
        self.slider1.reset()
        self.slider2.reset()
        self.slider3.reset()
        self.slider4.reset()
        pass



#test
if __name__ == "__main__":
     
     scara = SCARAPoseViewer()
     scara.slider()
