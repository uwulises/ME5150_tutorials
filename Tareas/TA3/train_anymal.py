import pybullet as p
import pybullet_data
import neat
import numpy as np
import csv

# Quadruped simulation parameters
QUADRUPED_URDF_PATH = "anymal/urdf/anymal.urdf"
LEG_JOINT_NUMBERS = [1, 2, 3, 6, 7, 8, 11, 12, 13, 16, 17, 18]
NUM_STEPS = 100

# NEAT parameters
NUM_INPUTS = 14         # Replace with the number of inputs based on your quadruped's state
NUM_OUTPUTS = 12        # Each leg has joint control
MAX_FORCES= [50] * 12   # Max force for joints
up_stairs_x = 4         # Target position

# NEAT training loop
NUM_GENERATIONS = 2
last_distance = 0
global generation

# Quadruped environment class
class QuadrupedEnv:
    def __init__(self):
        p.connect(p.GUI)  # or p.DIRECT for headless mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf") 
        p.loadURDF("objetos/stair.urdf", basePosition = [0, 0, 0], useFixedBase = True)
        p.setGravity(0, 0, -9.81)

        p.setTimeStep(1 / 400) 

        self.joint_ids = LEG_JOINT_NUMBERS
        self.quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition=[0, 0, 0.6])
        self.log_joints= np.array([])

    def hard_reset(self):
        self.log_joint_values()

        p.resetSimulation()
        p.loadURDF("plane.urdf") 
        p.loadURDF("objetos/stair.urdf", basePosition = [0, 0, 0], useFixedBase = True)
        p.setGravity(0, 0, -9.81)

        self.quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition = [0, 0, 0.6])
        self.log_joints = np.array([])


    def reset(self):
        p.setGravity(0, 0, -9.81)

        for index in self.joint_ids:
            p.resetJointState(self.quadruped, jointIndex = index, targetValue = 0)

    def get_quadruped_state(self):
        
        # Retrieve and return the state of the quadruped (e.g., position, orientation, velocities)
        # Implement this based on your specific needs

        joints_state = []
        for index in self.joint_ids:
            joints_state.append(p.getJointState(self.quadruped, jointIndex = index)[0])
        
        pos_x = p.getLinkState(self.quadruped, 0)[0][0]
        joints_state.append(pos_x)
        joints_state.append(up_stairs_x)
        return joints_state


    def get_fitness(self):
            
        # Compute and return the fitness score based on the performance of the quadruped
        # Implement this based on your specific objectives

        global last_distance
        fitness = 0.0 # Default value

        # Pose of the robot
        pos_x =         p.getLinkState(self.quadruped, 0)[0][0]
        pos_y =         p.getLinkState(self.quadruped, 0)[0][1]
        pos_z =         p.getLinkState(self.quadruped, 0)[0][2]
        orientation =   p.getLinkState(self.quadruped, 0)[1]
        angle_x =       p.getEulerFromQuaternion(orientation)[0]
        angle_y =       p.getEulerFromQuaternion(orientation)[1]
        angle_z =       p.getEulerFromQuaternion(orientation)[2]

        if (self.check_quadruped()):
            fitness = -1000
            last_distance = 0.0

        elif (pos_x > last_distance):
            fitness = pos_x * np.cos(angle_z) - abs(pos_y)
            last_distance = pos_x
            
        elif (round(pos_x,1) == round(last_distance,1)):
            fitness = -1
            last_distance = pos_x

        elif(round(pos_x) == up_stairs_x):
            exit()

        if abs(angle_z) < 0.1:
            fitness *= 0.7

        return fitness

    def check_quadruped(self):
        ori = p.getLinkState(self.quadruped, 0)[1]
        angle_x = p.getEulerFromQuaternion(ori)[0] * 180 / np.pi
        pos_y = p.getLinkState(self.quadruped, 0)[0][1]

        if (angle_x < -60 or angle_x > 60):
            quadruped_env.hard_reset()
            return True
        
        if (abs(pos_y) > 1.5):
            quadruped_env.hard_reset()
            return True
        
        return False
    
    def step(self, joints_state=np.zeros(12)):
        p.setJointMotorControlArray(self.quadruped, jointIndices = self.joint_ids, 
                                    controlMode = p.POSITION_CONTROL, 
                                    targetPositions = joints_state, forces=MAX_FORCES)
        self.log_joints = np.append(self.log_joints, self.get_quadruped_state())
        p.stepSimulation()

    def log_joint_values(self):
        with open('Tareas/TA3/joint_log.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.log_joints)
        

# NEAT fitness evaluation
def eval_genomes(genomes, config):
    for _, genome in genomes:
        quadruped_state = quadruped_env.get_quadruped_state()
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        fitness = 0.0

        # Evaluate the genome over a specified number of simulation steps
        for _ in range(NUM_STEPS):
            # Replace with appropriate inputs based on the quadruped's state
            inputs = quadruped_state
            outputs = net.activate(inputs)
            joints_states = outputs[:NUM_OUTPUTS]

            quadruped_env.step(joints_states)
            quadruped_state = quadruped_env.get_quadruped_state()
            fitness += quadruped_env.get_fitness()
            
        # Assign fitness to the genome
        genome.fitness = fitness

# NEAT setup
config_path = "Tareas/TA3/config.cfg"  # Replace with the path to your NEAT configuration file
config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)
population = neat.Population(config)
population.add_reporter(neat.StdOutReporter(True))
population.add_reporter(neat.StatisticsReporter())

# Quadruped environment initialization
quadruped_env = QuadrupedEnv()

for generation in range(NUM_GENERATIONS):
    
    best_genome = population.run(eval_genomes, 1)

    # Evaluate the best genome on a final test run
    quadruped_state = quadruped_env.get_quadruped_state()
    best_net = neat.nn.FeedForwardNetwork.create(best_genome, config)

    for _ in range(NUM_STEPS):
        # Replace with appropriate inputs based on the quadruped's state
        inputs = quadruped_state
        outputs = best_net.activate(inputs)
        joint_angles = outputs[:NUM_OUTPUTS]
        quadruped_env.step(joint_angles)
        quadruped_state = quadruped_env.get_quadruped_state()

quadruped_env.log_joint_values()