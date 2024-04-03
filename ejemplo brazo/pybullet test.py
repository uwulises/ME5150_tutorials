import pybullet as p
import pybullet_data

# Create a PyBullet simulation
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load the URDF file
robotId = p.loadURDF("roboticarm/urdftest.urdf")

# Fix the omnibase to the plane
planeId = p.loadURDF("plane.urdf")
constraintId = p.createConstraint(robotId, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])

# Run the simulation
while True:
    p.stepSimulation()
    p.getKeyboardEvents()


