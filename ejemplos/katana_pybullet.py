import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True #Para que el brazo no flote o se desplace

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("modelos/manipuladores/katana/katana.urdf", basePosition=[0, 0, 0],useFixedBase=useFixedBase)

p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Main simulation loop
while True:
    p.stepSimulation()
    time.sleep(1 / 240)

#p.stepSimulation()

