import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Load KR6 robot arm and table
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("brazos/kuka_model/kr6_2.urdf", basePosition = [0, 0, 0], useFixedBase = useFixedBase)
tableId = p.loadURDF("table/table.urdf", basePosition = [1.5, 0, 0], useFixedBase = useFixedBase)

width = 128
height = 128

fov = 60
aspect = width / height
near = 0.02
far = 1

view_matrix = p.computeViewMatrix([1., 0, 0.8], [1.6, 0, 2], [1, 0, 0])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

# Get depth values using the OpenGL renderer
images = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
depth_buffer_opengl = np.reshape(images[3], [width, height])
depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)

# # Get depth values using Tiny renderer
# images = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_TINY_RENDERER)
# depth_buffer_tiny = np.reshape(images[3], [width, height])
# depth_tiny = far * near / (far - (far - near) * depth_buffer_tiny)

# tool coordinate position
n_tcf = 5

# Create sliders for X, Y, Z, A, B, and C
x_slider = p.addUserDebugParameter("X", 0, 2, 1)
y_slider = p.addUserDebugParameter("Y", -1, 1, 0)
z_slider = p.addUserDebugParameter("Z", 0, 1.5, 1)
a_slider = p.addUserDebugParameter("A", -3.14, 3.14, 0)
b_slider = p.addUserDebugParameter("B", -3.14, 3.14, 0)
c_slider = p.addUserDebugParameter("C", -3.14, 3.14, 0)

# Run the simulation
while True:

    # get the current slider values
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = p.readUserDebugParameter(z_slider)
    a = p.readUserDebugParameter(a_slider)
    b = p.readUserDebugParameter(b_slider)
    c = p.readUserDebugParameter(c_slider)
    
    # target position
    xyz = [x,y,z]
    # target orientation
    ori = p.getQuaternionFromEuler([a,b,c])
    
    # calculate joint target
    target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex = n_tcf, targetPosition = xyz, targetOrientation = ori)
    p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions = target)
    p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    p.stepSimulation()
    time.sleep(0.1)
    