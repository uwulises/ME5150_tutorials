import numpy as np
import pybullet as p
import pybullet_data

# Define the DH parameters for the robot arm
a = [0, 0.4318, 0.0203, 0, 0, 0]
alpha = [np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0]
d = [0.333, 0, 0.316, 0, 0.384, 0.107]
theta = [0, 0, 0, 0, 0, 0]

# Define the end-effector position and orientation we want to achieve
target_pos = [1.2, 0, 1]
target_ori = [0, 0, 0, 1]  # Quaternion in [x, y, z, w] format

# Set up the PyBullet simulation environment and load the robot arm model
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("kr6_2.urdf", [0, 0, 0],useFixedBase=True)

# Define the inverse kinematics function using the Newton-Raphson method
def inverse_kinematics(target_pos, target_ori, max_iterations=100, tolerance=1e-2):
    # Initialize the joint angles
    theta = [0, 0, 0, 0, 0, 0]

    for i in range(max_iterations):
        # Compute the forward kinematics to get the end-effector position and orientation
        T = np.eye(4)
        for j in range(6):
            T = np.dot(T, p.getMatrixFromQuaternion(p.getLinkState(robotId, j)[5]))
            T[0:3, 3] = p.getLinkState(robotId, j)[4]
            T = np.dot(T, [[np.cos(theta[j]), -np.sin(theta[j])*np.cos(alpha[j]), np.sin(theta[j])*np.sin(alpha[j]), a[j]*np.cos(theta[j])],
                          [np.sin(theta[j]), np.cos(theta[j])*np.cos(alpha[j]), -np.cos(theta[j])*np.sin(alpha[j]), a[j]*np.sin(theta[j])],
                          [0, np.sin(alpha[j]), np.cos(alpha[j]), d[j]],
                          [0, 0, 0, 1]])

        # Compute the error between the target position and orientation and the actual position and orientation
        pos_error = np.array(target_pos) - T[0:3, 3]
        ori_error = np.array(target_ori) - p.getQuaternionFromMatrix(T[0:3, 0:3])

        # Compute the Jacobian matrix
        J = np.zeros((6, 6))
        for j in range(6):
            axis = p.getAxisInParentSpace(robotId, j)
            J[0:3, j] = np.cross(axis, T[0:3, 3] - p.getLinkState(robotId, j)[4])
            J[3:6, j] = axis

        # Compute the change in joint angles using the Newton-Raphson method
        delta_theta = np.linalg.solve(J, np.concatenate((pos_error, ori_error)))

        # Update the joint angles
        theta += delta_theta

        # Check if the error is below the tolerance
        if (np.linalg.norm(np.concatenate((pos_error, ori_error))) < tolerance):
            break

    return theta


joint_angles = inverse_kinematics(target_pos, target_ori)

for i in range(6):
    p.resetJointState(robotId, i, joint_angles[i])


while True:
    p.stepSimulation()