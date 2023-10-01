import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


class SCARAPoseViewer:

    def __init__(self, val_list = [0, 0, 0, 0]):
        # values_list := [hombro, codo, z, a], en grados excepto z[mm]
        self.values_list = val_list
        self.poses = np.array([])
        self.ax3 = None

    def get_pose_to_origin_pose_scara(self):
        # Dimensiones del links según la tesis del SCARA [mm]
        self.d1 = 360
        self.r1 = 330
        self.r2 = 668 - self.r1
        
        # Usen np.linalg.multi_dot([Z1, X1, ...., Zn, Xn]), para rotaciones: rot_axis(np.deg2rad(self.values_list[i]))

        # TODO: Implementar la cinematica directa del SCARA

        # Eje del origen
        origen = np.identity(4)
        
        # Primer link, del suelo al primer eje

        # Segundo link, del primer eje al segundo eje

        # Tercer link, del segundo eje al tercer eje (Z)
        
        # Cuarto link, del tercer eje al cuarto eje (A)
        

        # Generar las poses de todos los ejes

        # Agregar las poses de todos los ejes a la lista de poses
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

        self.draw_axes_tf(self.poses)

        # Creacion de sliders
        self.slider1_ax = plt.axes([0.2, 0.1, 0.65, 0.03])
        self.slider2_ax = plt.axes([0.2, 0.15, 0.65, 0.03])
        self.slider3_ax = plt.axes([0.2, 0.2, 0.65, 0.03])
        self.slider4_ax = plt.axes([0.2, 0.25, 0.65, 0.03])
        resetax = plt.axes([0.8, 0.025, 0.1, 0.04])

        self.slider1 = Slider(self.slider1_ax, 'Hombro', 10, 170, valinit=self.values_list[0])
        self.slider2 = Slider(self.slider2_ax, 'Codo', -130, 130, valinit=self.values_list[1])
        self.slider3 = Slider(self.slider3_ax, 'Z [mm]', 0, 180, valinit=self.values_list[2])
        self.slider4 = Slider(self.slider4_ax, 'A', 0, 360, valinit=self.values_list[3])
        
        # Agregar boton de reseteo
        button = plt.Button(resetax, 'Reset', color='white', hovercolor='0.975')
        button.on_clicked(self.reset)
        self.slider1.on_changed(self.update)
        self.slider2.on_changed(self.update)
        self.slider3.on_changed(self.update)
        self.slider4.on_changed(self.update)
        plt.show()
        
    def update(self, val):
        # Limpiar el plot
        self.ax3.cla()
        slider_values = np.array([self.slider1.val, self.slider2.val, self.slider3.val, self.slider4.val])
        self.values_list = slider_values
        self.get_pose_to_origin_pose_scara()
        self.draw_axes_tf(self.poses)
        pass

    # Crear un reset para los sliders
    def reset(self, event):
        self.slider1.reset()
        self.slider2.reset()
        self.slider3.reset()
        self.slider4.reset()
        pass

    def forward_kinematics(self, param_joints):
        """
        Parametros
            param_joints: parametros (angulo o distancia) de los joints en grados o mm.
                    numpy array de forma (4)

        Retorna
            pose: posicion del end effector en mm y orientacion en grados, restringido al workspace del robot.
                    numpy array de forma (4)             
        """
        self.values_list = param_joints
        self.get_pose_to_origin_pose_scara()
        
        # Obtener la orientacion del end effector
        x_rot = np.linalg.multi_dot([self.poses[-1], [1, 0, 0, 0]])
        angle = np.round(np.arctan2(x_rot[1], x_rot[0]),3)
    
        # TODO: Obtener la posicion del end effector desde la matriz de transformacion homogenea (self.poses)

        XYZ = np.array([0, 0, 0], dtype=float) # Reemplazar por columna correspondiente de la matriz de transformacion homogenea (self.poses)

        # Pose final del end effector
        pose = np.array([XYZ[0], XYZ[1], XYZ[2], np.rad2deg(angle)])
        return pose
    
    def inverse_kinematics(self, x, y, z, a):
        """
        Parametros
            x, y, z: posicion del end effector en mm
                    int o float        
            a: orientacion del end effector en grados
                    int o float
        Retorna
            params: parametros (angulo o distancia) de los joints en grados o mm.
                    numpy array de forma (4)

            Ejemplo: np.array[a, b, c, d], 10<=a<=170, -130<=b<=130, 0<=c<=180, 0<=d<=360
        """
        # TODO: Implementar la cinematica inversa del SCARA

        params = np.zeros(4) # Reemplazar por parametros de los 4 joints

        self.values_list = params
        self.get_pose_to_origin_pose_scara()
        return np.array(params, dtype = float)

if __name__ == "__main__":
    scara = SCARAPoseViewer()

    param_joints = [180, 0, 100, 90] # Ejemplo de parametros de los 4 joints, modificar por cualquier otro valor

    # No es necesario modificar esta parte
    dk = scara.forward_kinematics(param_joints)
    print('Cinematica directa:', dk)
    ik = scara.inverse_kinematics(dk[0], dk[1], dk[2], dk[3])
    print('Cinematica inversa:', ik)

    # param_joints debe ser igual o equivalente a ik
    scara.slider()

    