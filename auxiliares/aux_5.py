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

#largos de los eslabones del SCARA, en mm
L1=330
L2=336

"""
Se crea la función que calcula la matriz jacobiana de una funcion
f: función que se desea calcular la matriz jacobiana (vector)
x: punto en el que se desea calcular la matriz jacobiana (vector)
"""
def jacobian(f, x, epsilon=1e-1): #epsilon es el error admisible
    n = len(x)
    jacobian_matrix = np.zeros((n, n))
    f_x = f(x)
    
    for i in range(n):
        perturbation = np.zeros(n)
        perturbation[i] = epsilon
        f_x_plus = f(x + perturbation)
        jacobian_matrix[:, i] = (f_x_plus - f_x) / epsilon
        
    return jacobian_matrix



# def NewtonRaphson(f, theta0, target_pos, tol=1e-3, max_iter=100):
#     theta = theta0 #theta0 es el initial guess
#     for _ in range(max_iter):
#         #calcular jacobiano con la funcion anterior
#         jacobiano_val = 
#         #primero calcular la matriz inversa
#         try:
#         except np.linalg.LinAlgError:
#             #si no es invertible, habrá un error, en ese caso se calcula la pseudoinversa
#         #aplicar metodo newton raphson
#         if np.abs(np.linalg.det(jacobiano_val))>tol:
        
#         #si el determinante es cercano a 0, se agrega ruido al vector theta, para que no diverja 
#         else:
#             theta_new = theta + np.random.rand(len(theta)) #se mueve theta aleatoriamente
#         #si la norma de la diferencia entre theta_new y theta es menor que la tolerancia, se retorna theta_new

#         #condicion de convergencia
#         if np.linalg.norm(theta_new - theta) < tol:
#             return theta_new 
#         theta = theta_new
#     raise ValueError("No se pudo encontrar la solución después de %d iteraciones" % max_iter)

#funciones de la cinemática directa
def F(theta):
    f1=L1*np.cos(theta[0])+L2*np.cos(theta[0]+theta[1])
    f2=L1*np.sin(theta[0])+L2*np.sin(theta[0]+theta[1])
    f3=#agregar funcion en z
    return np.array([f1, f2, f3])


"""
Obtener los parámetros (resolucion cinematica inversa)
NewtonRapson(funcion, punto inicial, posicion deseada)
punto incial representa los valores de los joints, como semilla,
en este caso son 3: dos rotaciones y uno de traslacion
posicion deseada es un vector de 3: x,y,z en el espacio cartesiano

"""
#entregar posicion deseada x,y,z
posicion_deseada=np.array([])
#q= NewtonRaphson(F, np.array([np.pi/4, np.pi/4, 150]), posicion_deseada)

#mover el robot a las posiciones calculadas usando pybullet, con c. directa
def mover_parametros_obtenidos(q):
    """
    se realiza una transformacion de los angulos para que estén restringidos
    a un rango de -pi a pi y la traslacion a metros
    """
    q_rad_y_metros=[np.mod(q[0]+np.pi,2*np.pi)-np.pi, 
                 np.mod(q[1]+np.pi,2*np.pi)-np.pi, 
                 (q[2]/1000)]
    for i in range(len(q)):
        p.setJointMotorControl2(robotId, i, p.POSITION_CONTROL, targetPosition=q_rad_y_metros[i])


#Cinemática inversa de pybullet, para comprobar los resultados
def cin_inv_pybullet(x, y, z):
    xyz=[x, y, z]
    target=p.calculateInverseKinematics(robotId, n_tcf, targetPosition=xyz)
    for i in range(len(target)):
        p.setJointMotorControl2(robotId, i, p.POSITION_CONTROL, targetPosition=target[i])


def main():
    while True:
        """
        comentar y descomentar para usar el método o mostrar la simulación
        """
        #a la función que implementa el método se le pasa el array entregado
        #por NewtonRaphson
        mover_parametros_obtenidos(q)

        """
        para la simulación en pybullet se transforma la posición deseada (orginalmente
        en mm) a metros, porque pybullet trabaja con metros
        """
        #cin_inv_pybullet(posicion_deseada[0]/1000, posicion_deseada[1]/1000, posicion_deseada[2]/1000)
        

if __name__ == "__main__":
    
    main()