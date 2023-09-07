import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Load SCARA robot arm and table
planeId = p.loadURDF("plane.urdf")
baseId = p.loadURDF("../modelos/manipuladores/scara/base_scara.urdf",
                     basePosition=[0, 0, 0.69], useFixedBase=useFixedBase)
table2Id = p.loadURDF("../modelos/manipuladores/scara/base_scara.urdf",
                     basePosition=[0.8, 0, 0.69], useFixedBase=useFixedBase)
robotId = p.loadURDF("../modelos/manipuladores/scara/scara.urdf", basePosition = [0, 0, 0.69], useFixedBase = useFixedBase)
cubeId = p.loadURDF("../modelos/objetos/cubo.urdf", basePosition = [0.6, 0.2, 0.7])
# tool coordinate position
n_tcf = 2

# Add sliders to control the position of the cube
x_slider = p.addUserDebugParameter("x", 0.5, 0.8,0.6)
y_slider = p.addUserDebugParameter("y", 0, 0.5, 0)

### funcion para obtener imagen de camara simulada
def get_img_cam(width=240, height=240, fov=60, near=0.02, far=2, camposition=[1, 0, 1.5],distance=0.1,yaw=0,pitch=-90,roll=0):
    aspect = width / height
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition,distance,yaw,pitch,roll,upAxisIndex=2)
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    w_img, h_img, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
    depth_buffer_opengl = np.reshape(depthImg, [width, height])
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)
    return rgbaImg, segImg, depthImg


def find_object_center(segmented_image, object_id):

    # Convert the segmented image to a numpy array for easier manipulation
    segmented_array = np.array(segmented_image)
    # Find the pixels belonging to the object ID
    object_pixels = np.where(segmented_array == object_id)
    
    # Calculate the center coordinates
    center_x = np.mean(object_pixels[1])
    center_y = np.mean(object_pixels[0])
    if np.isnan(center_x).any() or np.isnan(center_y).any():
       return [0,0]
    return int(center_x), int(center_y)

    
#function to move the cubeId to the target position using sliders
def move_cube():
    # get the current slider values
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = 0.715
    # target position
    xyz = [x, y, z]
    p.resetBasePositionAndOrientation(cubeId, xyz, [0, 0, 0, 1])
    p.stepSimulation()



# Run the simulation
while True:
    #
    move_cube()

    img_RGB, img_segmentada, img_depth = get_img_cam()
    center_segm_id = find_object_center(img_segmentada, cubeId)

    
    # calculate joint target (center + offset pixel & world offset)
    target_obj_x = (center_segm_id[0]-120)/215 + 1 
    target_obj_y = -(center_segm_id[1]-120)/215

    xyz = [target_obj_x, target_obj_y, 0.75]

    target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex = n_tcf, targetPosition = xyz)
    p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL, targetPositions = target)

    #handle position point over rgb image
    print("Posicion robot: {} , Centro en px objeto: {}".format(xyz, center_segm_id))
    cv2.circle(img_RGB, (center_segm_id[0], center_segm_id[1]), radius=3, color=(0,0,255), thickness=-1)
    cv2.imshow("Seg", img_RGB)
    cv2.waitKey(1)
    
    p.stepSimulation()

    