import pybullet as p
import pybullet_data
import time
import math
import pygame

# Inicializar PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Para encontrar los archivos URDF en pybullet_data

# Crear el suelo
floor = p.loadURDF("plane.urdf")

# Crear el robot (base móvil)
robot = p.loadURDF("modelos/base_movil/omnibase.urdf", [0, 0, 0.05]) 
# Establecer gravedad y paso de tiempo
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

action_threshold = 0.1
min_time_between_actions = 1.0  # Definir aquí el tiempo mínimo entre acciones en segundos
last_action_time = time.time()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    x_axis, y_axis, x_axis_rot = joystick.get_axis(2), joystick.get_axis(3), joystick.get_axis(0)
    boton_stop, boton_exit = joystick.get_button(2), joystick.get_button(3)
    
    current_time = time.time()
    action = ""
    
    desfase= 0.785398 #45 grados en radianes
    velocidad=5.0

    base_position, base_orientation = p.getBasePositionAndOrientation(robot)
    roll, pitch, yaw = p.getEulerFromQuaternion(base_orientation)

    if abs(x_axis) > action_threshold and abs(y_axis) > action_threshold: 
        if x_axis  > 0 and y_axis > 0: #diagonal derecha arriba
            linear_velocity = [-2.0*math.cos(yaw+desfase), -2.0*math.sin(yaw+desfase), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif x_axis > 0 and y_axis < 0: #diagonal derecha abajo
            linear_velocity = [2.0*math.cos(yaw-desfase), 2.0*math.sin(yaw-desfase), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif x_axis < 0 and y_axis > 0:   #diagonal izquierda arriba
            linear_velocity = [-2.0*math.cos(yaw-desfase), -2.0*math.sin(yaw-desfase), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif x_axis < 0 and y_axis < 0:  #diagonal izquierda abajo
            linear_velocity = [2.0*math.cos(yaw+desfase), 2.0*math.sin(yaw+desfase), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
    elif abs(x_axis) > action_threshold:
        if x_axis > 0: 
            linear_velocity = [2.0*math.sin(yaw), -2.0*math.cos(yaw), 0.0]
            angular_velocity = [0.0, 0.0, 0.0] 
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        else: 
            linear_velocity = [-2.0*math.sin(yaw), 2.0*math.cos(yaw), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
    elif abs(y_axis) > action_threshold:    
        if y_axis > 0:
            linear_velocity = [-2.0*math.cos(yaw), -2.0*math.sin(yaw), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif y_axis < 0:
            linear_velocity = [2.0*math.cos(yaw), 2.0*math.sin(yaw), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
    elif abs(x_axis_rot) > action_threshold:
        if x_axis_rot > 0:
            linear_velocity = [0.0, 0.0, 0.0]
            angular_velocity = [0.0, 0.0, -8.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        else:
            linear_velocity = [0.0, 0.0, 0.0]
            angular_velocity = [0.0, 0.0, 8.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)

    if boton_stop:
        action = "stop"
        print("Botón stop")

    if action:
        if current_time - last_action_time > min_time_between_actions:
            print("Acción:", action, "x_axis:", x_axis, "y_axis:", y_axis)
            last_action_time = current_time

    if boton_exit:
        exit()

    p.stepSimulation()
    time.sleep(0.01)
