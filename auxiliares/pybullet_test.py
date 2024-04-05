import pybullet as p
import pybullet_data

# Create a PyBullet simulation
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
# Load the URDF file 
robotId = p.loadURDF("../modelos/manipuladores/twolinks/urdftest.urdf", basePosition = [0, 0, 0.0125], useFixedBase = True)

# Fix the omnibase to the plane
print("algo")

print("ID", robotId)


# Run the simulation
while True:
    p.stepSimulation()
    p.getKeyboardEvents()

