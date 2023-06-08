import pybullet as p
import pybullet_data
import time
import neat
import random
import numpy as np
import gzip
import pickle

# Quadruped simulation parameters
QUADRUPED_URDF_PATH = "../anymal/urdf/anymal.urdf"
NUM_LEGS = 1
LEG_JOINT_NAMES = ["LF_HAA", "LF_HFE", "LF_KFE"]  # Replace with actual joint names
LEG_JOINT_NUMBERS = [1, 2, 3, 6, 7, 8, 11, 12, 13, 16, 17, 18]
NUM_STEPS = 100

# NEAT parameters
NUM_INPUTS = 12  # Replace with the number of inputs based on your quadruped's state
NUM_OUTPUTS = 12  # Each leg has joint control

# NEAT training loop
NUM_GENERATIONS = 10

global generation

# Quadruped environment class
class QuadrupedEnv:
    def __init__(self): 
        p.connect(p.GUI)  # or p.DIRECT for headless mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")  # Load the ground plane
        p.setGravity(0, 0, -9.81)

        self.quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition=[0, 0, 0.8])
        self.joint_ids = LEG_JOINT_NUMBERS

        p.setTimeStep(1 / 240)  # Control the simulation speed
    
    def hard_reset(self):
        p.resetSimulation()
        p.loadURDF("plane.urdf")  # Load the ground plane
        self.quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition=[0, 0, 0.8])
        p.setGravity(0, 0, -9.81)

    def reset(self):
        # Reset the simulation
        p.setGravity(0, 0, -9.81)
        
        for index in self.joint_ids:
            p.resetJointState(self.quadruped,jointIndex=index,targetValue=0)

    def step(self, joint_angles=np.zeros(12)):
        i = 0
        for index in self.joint_ids:
            p.setJointMotorControl2(self.quadruped,jointIndex=index,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[i])
            i+=1
        p.stepSimulation()

        

    def get_quadruped_state(self):
        # Retrieve and return the state of the quadruped (e.g., position, orientation, velocities)
        # Implement this based on your specific needs

        joints_state = []
        for index in self.joint_ids:
            joints_state.append(p.getJointState(self.quadruped,jointIndex=index)[0])

        return joint_angles

    def get_fitness(self):
        # Compute and return the fitness score based on the performance of the quadruped
        # Implement this based on your specific objectives
        fitness=0.0
        #TODO 

        distance_x= p.getLinkState(self.quadruped,0)[0][0]
        distance_y= p.getLinkState(self.quadruped,0)[0][1]
        ori = p.getLinkState(self.quadruped,0)[1]
        angle_x = p.getEulerFromQuaternion(ori)[0]*180/np.pi
        angle_y = p.getEulerFromQuaternion(ori)[1]*180/np.pi
        angle_z = p.getEulerFromQuaternion(ori)[2]*180/np.pi
        orientation_deg = [angle_x, angle_y , angle_z]

        self.check_ori_x()

        fitness = distance_x


        return fitness 
    


    def check_ori_x(self):
        ori = p.getLinkState(self.quadruped,0)[1]
        angle_x = p.getEulerFromQuaternion(ori)[0]*180/np.pi
        if (angle_x<-60 or angle_x>60):
            quadruped_env.hard_reset()
            return True
        return False
    
def save_checkpoint(config, population, generation):
    """ Save the current simulation state. """
    filename = 'best_genome.neat-checkpoint-'
    print("Saving checkpoint to {0}".format(filename))

    with gzip.open(filename, 'w', compresslevel=5) as f:
        data = (generation, config, population, 0, random.getstate())
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)

# NEAT fitness evaluation
def eval_genomes(genomes, config):
    for genome_id, genome in genomes:
        quadruped_env.reset()
        quadruped_state = quadruped_env.get_quadruped_state()
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        fitness = 0.0

        # Evaluate the genome over a specified number of simulation steps
        for _ in range(NUM_STEPS):
            inputs = quadruped_state  # Replace with appropriate inputs based on the quadruped's state
            outputs = net.activate(inputs)
            joint_angles = outputs[:NUM_OUTPUTS]
            quadruped_env.step(joint_angles)
            quadruped_state = quadruped_env.get_quadruped_state()

            fitness += quadruped_env.get_fitness()
            

        # Assign fitness to the genome
        genome.fitness = fitness
        if (quadruped_env.check_ori_x()):
            genome.fitness = -10
    

# NEAT setup
config_path = "config.cfg"  # Replace with the path to your NEAT configuration file
config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)
population = neat.Population(config)
population.add_reporter(neat.StdOutReporter(True))
stats = neat.StatisticsReporter()
population.add_reporter(stats)

# Quadruped environment initialization
quadruped_env = QuadrupedEnv()


for generation in range(NUM_GENERATIONS):

    print(f"Generation {generation + 1}/{NUM_GENERATIONS}")
    best_genome = population.run(eval_genomes, 1)

    # Evaluate the best genome on a final test run
    quadruped_env.reset()
    quadruped_state = quadruped_env.get_quadruped_state()
    best_net = neat.nn.FeedForwardNetwork.create(best_genome, config)

    for _ in range(NUM_STEPS):
        inputs = quadruped_state  # Replace with appropriate inputs based on the quadruped's state
        outputs = best_net.activate(inputs)
        joint_angles = outputs[:NUM_OUTPUTS]
        quadruped_env.step(joint_angles)
        quadruped_state = quadruped_env.get_quadruped_state()
    quadruped_env.check_ori_x()
    save_checkpoint(config, population, generation)