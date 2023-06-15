import pybullet as p
import pybullet_data
import numpy as np

# NEAT parameters
NUM_INPUTS = 14  # Replace with the number of inputs based on your quadruped's state
NUM_OUTPUTS = 12  # Each leg has joint control
QUADRUPED_URDF_PATH = "anymal/urdf/anymal.urdf"
LEG_JOINT_NUMBERS = [1, 2, 3, 6, 7, 8, 11, 12, 13, 16, 17, 18]
MAX_FORCE = [50]*12 #Lista de máximas fuerzas 
up_stairs_x=4

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.loadURDF("objetos/stair.urdf", basePosition = [0, 0, 0], useFixedBase = True)

p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 400)

quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition=[0, 0, 0.6])

arr = np.loadtxt("Tareas/TA3/joint_log.csv", delimiter=",", dtype=float)

robot_states = np.array(arr).reshape((-1,14))
joint_states = robot_states[:,:-2]

for target in joint_states:
    p.setJointMotorControlArray(quadruped, jointIndices = LEG_JOINT_NUMBERS, controlMode=p.POSITION_CONTROL, targetPositions = target, forces = MAX_FORCE)
    p.stepSimulation()
