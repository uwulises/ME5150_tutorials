import numpy as np
import pybullet as p
import pybullet_data

#Simulacion en pybullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

# Load SCARA robot arm and table
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("../modelos/manipuladores/scara/base_scara.urdf",
                     basePosition=[0, 0, 0.69], useFixedBase=useFixedBase)
initialori = p.getQuaternionFromEuler([0, 0, np.deg2rad(90)]) # initial orientation of the robot
robotId = p.loadURDF("../modelos/manipuladores/scara/scara.urdf",
                     basePosition=[0, 0, 0.69], baseOrientation=initialori,useFixedBase=useFixedBase)
# tool coordinate position
n_tcf = 2


L1=330
L2=336

def jacobian(f, x, epsilon=1e-1):
    """
    Calcula el jacobiano de la función f en el punto x utilizando diferencias finitas.
    
    Args:
    - f: La función para la cual se calculará el jacobiano.
    - x: El punto en el cual se evaluará el jacobiano.
    - epsilon: La pequeña perturbación para calcular las diferencias finitas.
    
    Returns:
    - El jacobiano de f en el punto x.
    """
    n = len(x)
    jacobian_matrix = np.zeros((n, n))
    f_x = f(x)
    
    for i in range(n):
        perturbation = np.zeros(n)
        perturbation[i] = epsilon
        f_x_plus = f(x + perturbation)
        jacobian_matrix[:, i] = (f_x_plus - f_x) / epsilon
        
    return jacobian_matrix

def NewtonRapson(f, x0, target_pos, tol=1e-3, maxiter=100):
    x = x0
    for _ in range(maxiter):
        try:
            jacobiano_val=jacobian(f, x)
        except:
            #usar la pseudo inversa si es singular
            jacobiano_val=np.linalg.pinv(jacobian(f, x))
            
        if np.abs(np.linalg.det(jacobiano_val))>0.001:
            x_new = x - np.linalg.inv(jacobiano_val).dot(f(x) - target_pos)
        else:
            x_new = x + np.random.rand(len(x))
        if np.linalg.norm(x_new - x) < tol:
            return x_new 
        x = x_new
    raise ValueError("No se pudo encontrar la solución después de %d iteraciones" % maxiter)

def F(x):
    f1=L1*np.cos(x[0])+L2*np.cos(x[0]+x[1])
    f2=L1*np.sin(x[0])+L2*np.sin(x[0]+x[1])
    f3=100+x[2]
    return np.array([f1, f2, f3])

#entregar posicion deseada
posicion_deseada=np.array([250, 70, 150])
#obtener los valores de las articulaciones
q= NewtonRapson(F, np.array([np.pi/4, np.pi/4,150]), posicion_deseada)

#mover el robot a las posiciones calculadas
def mover_parametros_obtenidos(q):
    q_rad_y_metros=[np.mod(q[0]+np.pi,2*np.pi)-np.pi, 
                 np.mod(q[1]+np.pi,2*np.pi)-np.pi, 
                 (q[2]/1000)]
    for i in range(len(q)):
        p.setJointMotorControl2(robotId, i, p.POSITION_CONTROL, targetPosition=q_rad_y_metros[i])
print(q)

#implementar la cinemática inversa de pybullet
def cin_inv_pybullet(x, y, z):
    xyz=[x, y, z]
    target=p.calculateInverseKinematics(robotId, n_tcf, targetPosition=xyz)
    for i in range(len(target)):
        p.setJointMotorControl2(robotId, i, p.POSITION_CONTROL, targetPosition=target[i])
print(posicion_deseada)

def main():
    while True:
        #Selects from Inverse or Forward Kinematics
        mover_parametros_obtenidos(q)
        #cin_inv_pybullet(posicion_deseada[0]/1000, posicion_deseada[1]/1000, posicion_deseada[2]/1000)
        

if __name__ == "__main__":
    
    main()