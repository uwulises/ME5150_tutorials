import pybullet as p
import pybullet_data
import time
import keyboard
import math

# Inicializar PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Para encontrar los archivos URDF en pybullet_data

# Crear el suelo
floor = p.loadURDF("plane.urdf")

# Crear el robot (base móvil)
robot = p.loadURDF("ME5150_tutorials/modelos/base_movil/omnibase.urdf", [0, 0, 0.05]) 

# Establecer gravedad y paso de tiempo
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

while True:
    key_event = keyboard.read_event()
    desfase= 0.785398
    if key_event.event_type == 'down':
        # Obtener la posición y orientación actual de la base
        base_position, base_orientation = p.getBasePositionAndOrientation(robot)
        
        # Obtener los ángulos de Euler a partir del cuaternión de orientación
        roll, pitch, yaw = p.getEulerFromQuaternion(base_orientation)
        
        # Mapa de eventos de teclado a comandos
        if key_event.name == 'q':
            linear_velocity = [2.0*math.cos(yaw), 2.0*math.sin(yaw), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)

        elif key_event.name == 's':
            linear_velocity = [-2.0*math.cos(yaw), -2.0*math.sin(yaw), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif key_event.name == 'a':
            linear_velocity = [0.0, 0.0, 0.0]
            angular_velocity = [0.0, 0.0, 8.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif key_event.name == 'd':
            linear_velocity = [0.0, 0.0, 0.0]
            angular_velocity = [0.0, 0.0, -8.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif key_event.name == 'z': #lateral derecha
            linear_velocity = [2.0*math.sin(yaw), -2.0*math.cos(yaw), 0.0]
            angular_velocity = [0.0, 0.0, 0.0] 
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif key_event.name == 'x': #lateral izquiera
            linear_velocity = [-2.0*math.sin(yaw), 2.0*math.cos(yaw), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif key_event.name == 'm': #movimiento diagonal derecha arriba
            linear_velocity = [2.0*math.cos(yaw+desfase), 2.0*math.sin(yaw+desfase), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif key_event.name == 'n': #movimiento diagonal derecha abajo
            linear_velocity = [2.0*math.cos(yaw-desfase), 2.0*math.sin(yaw-desfase), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif key_event.name == 'b': #movimiento diagonal izquierda arriba
            linear_velocity = [-2.0*math.cos(yaw-desfase), -2.0*math.sin(yaw-desfase), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        elif key_event.name == 'c': #movimiento diagonal izquierda abajo
            linear_velocity = [-2.0*math.cos(yaw+desfase), -2.0*math.sin(yaw+desfase), 0.0]
            angular_velocity = [0.0, 0.0, 0.0]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)
        else:
            command = 'stop'

    p.stepSimulation()
    time.sleep(0.01)
