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
    desfase= 0.785398 #45 grados en radianes
    velocidad=5.0
    if key_event.event_type == 'down':
        base_position, base_orientation = p.getBasePositionAndOrientation(robot)
        roll, pitch, yaw = p.getEulerFromQuaternion(base_orientation)

        # Mapa de eventos de teclado a comandos
        key_commands = {
            'q': (velocidad * math.cos(yaw), velocidad * math.sin(yaw), 0.0, 0.0, 0.0, 0.0), #adelante
            's': (-velocidad * math.cos(yaw), -velocidad * math.sin(yaw), 0.0, 0.0, 0.0, 0.0), #atras
            'a': (0.0, 0.0, 0.0, 0.0, 0.0, 3*velocidad), #giro izquierda
            'd': (0.0, 0.0, 0.0, 0.0, 0.0, -3*velocidad),  #giro derecha
            'z': (velocidad * math.sin(yaw), -velocidad * math.cos(yaw), 0.0, 0.0, 0.0, 0.0), #lateral derecha
            'x': (-velocidad * math.sin(yaw), velocidad * math.cos(yaw), 0.0, 0.0, 0.0, 0.0),   #lateral izquierda
            'm': (velocidad * math.cos(yaw + desfase), velocidad * math.sin(yaw + desfase), 0.0, 0.0, 0.0, 0.0), #diagonal derecha arriba
            'n': (velocidad * math.cos(yaw - desfase), velocidad * math.sin(yaw - desfase), 0.0, 0.0, 0.0, 0.0), #diagonal izquierda arriba
            'b': (-velocidad * math.cos(yaw - desfase), -velocidad * math.sin(yaw - desfase), 0.0, 0.0, 0.0, 0.0), #diagonal derecha abajo
            'c': (-velocidad * math.cos(yaw + desfase), -velocidad * math.sin(yaw + desfase), 0.0, 0.0, 0.0, 0.0), #diagonal izquierda abajo
        }

        command = key_commands.get(key_event.name)

        if command:
            linear_velocity = command[:3]
            angular_velocity = command[3:]
            p.resetBaseVelocity(robot, linear_velocity, angular_velocity)

    p.stepSimulation()
    time.sleep(0.01)

