import pybullet as p
import pybullet_data
import time
import math
import pygame
import cv2
import numpy as np
import threading

# Inicializar PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Para encontrar los archivos URDF en pybullet_data

# Crear el suelo
floor = p.loadURDF("plane.urdf")

# Crear el robot (base móvil)
omnibase = p.loadURDF("modelos/base_movil/omnibase.urdf", [0, 0, 0.05]) 
# Establecer gravedad y paso de tiempo
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

class remote_control:
    def __init__(self, joystick, robot):
        self.joystick = joystick
        self.robot = robot 
        self.action_threshold = 0.1
        self.min_time_between_actions = 1.0
        self.joystick.init()

    def movimiento(self):
        last_action_time = time.time()

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()

            x_axis, y_axis, x_axis_rot = self.joystick.get_axis(2), self.joystick.get_axis(3), self.joystick.get_axis(0)
            boton_stop, boton_exit = self.joystick.get_button(2), self.joystick.get_button(3)

            current_time = time.time()
            action = ""

            desfase = 0.785398  # 45 grados en radianes

            base_position, base_orientation = p.getBasePositionAndOrientation(self.robot)
            roll, pitch, yaw = p.getEulerFromQuaternion(base_orientation)
            if abs(x_axis) > self.action_threshold and abs(y_axis) > self.action_threshold:
                if x_axis > 0 and y_axis > 0:  # diagonal derecha arriba
                    linear_velocity = [-2.0 * math.cos(yaw + desfase), -2.0 * math.sin(yaw + desfase), 0.0]
                    angular_velocity = [0.0, 0.0, 0.0]
                    p.resetBaseVelocity(self.robot, linear_velocity, angular_velocity)
                elif x_axis > 0 and y_axis < 0:  # diagonal derecha abajo
                    linear_velocity = [2.0 * math.cos(yaw - desfase), 2.0 * math.sin(yaw - desfase), 0.0]
                    angular_velocity = [0.0, 0.0, 0.0]
                    p.resetBaseVelocity(self.robot, linear_velocity, angular_velocity)
                elif x_axis < 0 and y_axis > 0:  # diagonal izquierda arriba
                    linear_velocity = [-2.0 * math.cos(yaw - desfase), -2.0 * math.sin(yaw - desfase), 0.0]
                    angular_velocity = [0.0, 0.0, 0.0]
                    p.resetBaseVelocity(self.robot, linear_velocity, angular_velocity)
                elif x_axis < 0 and y_axis < 0:  # diagonal izquierda abajo
                    linear_velocity = [2.0 * math.cos(yaw + desfase), 2.0 * math.sin(yaw + desfase), 0.0]
                    angular_velocity = [0.0, 0.0, 0.0]
                    p.resetBaseVelocity(self.robot, linear_velocity, angular_velocity)
            elif abs(x_axis) > self.action_threshold:
                if x_axis > 0:
                    linear_velocity = [2.0 * math.sin(yaw), -2.0 * math.cos(yaw), 0.0]
                    angular_velocity = [0.0, 0.0, 0.0]
                    p.resetBaseVelocity(self.robot, linear_velocity, angular_velocity)
                else:
                    linear_velocity = [-2.0 * math.sin(yaw), 2.0 * math.cos(yaw), 0.0]
                    angular_velocity = [0.0, 0.0, 0.0]
                    p.resetBaseVelocity(self.robot, linear_velocity, angular_velocity)
            elif abs(y_axis) > self.action_threshold:
                if y_axis > 0:
                    linear_velocity = [-2.0 * math.cos(yaw), -2.0 * math.sin(yaw), 0.0]
                    angular_velocity = [0.0, 0.0, 0.0]
                    p.resetBaseVelocity(self.robot, linear_velocity, angular_velocity)
                elif y_axis < 0:
                    linear_velocity = [2.0 * math.cos(yaw), 2.0 * math.sin(yaw), 0.0]
                    angular_velocity = [0.0, 0.0, 0.0]
                    p.resetBaseVelocity(self.robot, linear_velocity, angular_velocity)
            elif abs(x_axis_rot) > self.action_threshold:
                if x_axis_rot > 0:
                    linear_velocity = [0.0, 0.0, 0.0]
                    angular_velocity = [0.0, 0.0, -8.0]
                    p.resetBaseVelocity(self.robot, linear_velocity, angular_velocity)
                else:
                    linear_velocity = [0.0, 0.0, 0.0]
                    angular_velocity = [0.0, 0.0, 8.0]
                    p.resetBaseVelocity(self.robot, linear_velocity, angular_velocity)

            if boton_stop:
                action = "stop"
                print("Botón stop")

            if action:
                print("current_time:", current_time, "last_action_time:", last_action_time)
                if current_time - last_action_time > self.min_time_between_actions:
                    print("Acción:", action, "x_axis:", x_axis, "y_axis:", y_axis)
                    last_action_time = current_time

            if boton_exit:
                break


class camara:
    def __init__(self, width=320, height=240, fov=60, near=0.01, far=5, camposition=[1, 0, 1.5],distance=0.0035,yaw=0,pitch=-90,roll=90):
        self.width = width
        self.height = height
        self.fov = fov
        self.near = near
        self.far = far
        self.camposition = camposition
        self.distance = distance
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
    def get_img_cam(self,camposition,yaw,pitch,roll):
        self.camposition = camposition
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        aspect = self.width / self.height
        view_matrix = p.computeViewMatrixFromYawPitchRoll(self.camposition,self.distance,self.yaw,self.pitch,self.roll,upAxisIndex=1)
        projection_matrix = p.computeProjectionMatrixFOV(self.fov, aspect, self.near, self.far)
        w_img, h_img, rgbaImg, depthImg, segImg = p.getCameraImage(self.width, self.height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
        depth_buffer_opengl = np.reshape(depthImg, [self.width, self.height])
        rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)

        return rgbaImg, segImg, depthImg

def run_controller(joystick, robot):
    controller = remote_control(joystick, robot)
    controller.movimiento()

def main():
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    control = threading.Thread(target=run_controller, args=(joystick, omnibase,))
    control.start()

    camera= camara()
    while True:
        #ARREGLAR LA POSICION Y ORIENTACIÓN DE LA CAMARA
        pose_base = p.getBasePositionAndOrientation(omnibase, 0)
        posicion_base = pose_base[0]
        posicion_base= posicion_base + np.array([0,0,0.05])
        roll, pitch, yaw = p.getEulerFromQuaternion(pose_base[1])
        img, seg, depth = camera.get_img_cam(camposition=posicion_base,roll=roll,pitch=pitch,yaw=yaw)
        cv2.imshow("img", img)
        #cv2.imshow("seg", seg)
        cv2.imshow("depth", depth)
        cv2.waitKey(1)
        p.stepSimulation()
        time.sleep(0.01)
        

if __name__ == "__main__":
    main()