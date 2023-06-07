import pybullet as p
import pybullet_data
import time
import neat
import random
import numpy as np

# Quadruped simulation parameters
QUADRUPED_URDF_PATH = "../anymal/urdf/anymal.urdf"
NUM_LEGS = 1
LEG_JOINT_NAMES = ["LF_HAA", "LF_HFE", "LF_KFE"]  # Replace with actual joint names
LEG_JOINT_NUMBERS = [1,2,3,6,7,8]

MAX_JOINT_FORCE = 80  # Maximum joint force applied by the motors
NUM_STEPS = 30
# NEAT parameters
NUM_INPUTS = 12  # Replace with the number of inputs based on your quadruped's state
#NUM_OUTPUTS = NUM_LEGS * len(LEG_JOINT_NAMES)  # Each leg has joint control
NUM_OUTPUTS = 12  # Each leg has joint control
# NEAT training loop
NUM_GENERATIONS = 100
# Quadruped environment class
class QuadrupedEnv:
    def __init__(self):
        p.connect(p.GUI)  # or p.DIRECT for headless mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")  # Load the ground plane
        self.quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition=[0, 0, 0.8])
        p.setGravity(0, 0, -9.81)
        self.joint_ids = []
        # for leg_id in range(NUM_LEGS):
        #     for joint_name in LEG_JOINT_NAMES:
        #         joint_id = p.getJointInfo(self.quadruped, leg_id, joint_name)
        #         self.joint_ids.append(joint_id[0])
        self.joint_ids = LEG_JOINT_NUMBERS

        p.setTimeStep(1 / 2000)  # Control the simulation speed

    def reset(self):
        # Reset the simulation
        p.setGravity(0, 0, -9.81)
        #TODO modify to call every joint
        p.resetJointState(self.quadruped,jointIndex=1,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=2,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=3,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=6,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=7,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=8,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=11,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=12,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=13,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=16,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=17,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=18,targetValue=0)

    def step(self, joint_angles=[0,0,0,0,0,0,0,0,0,0,0,0]):
        #TODO modify to call every joint
        p.setJointMotorControl2(self.quadruped,jointIndex=1,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[0],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=2,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[1],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=3,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[2],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=6,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[3],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=7,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[4],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=8,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[5],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=11,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[6],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=12,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[7],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=13,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[8],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=16,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[9],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=17,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[10],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=18,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[11],force=MAX_JOINT_FORCE)
        p.stepSimulation()

        

    def get_quadruped_state(self):
        # Retrieve and return the state of the quadruped (e.g., position, orientation, velocities)
        # Implement this based on your specific needs

        #TODO 
        J1_state = p.getJointState(self.quadruped,jointIndex=1)[0]
        J2_state = p.getJointState(self.quadruped,jointIndex=2)[0]
        J3_state = p.getJointState(self.quadruped,jointIndex=3)[0]
        J6_state = p.getJointState(self.quadruped,jointIndex=6)[0]
        J7_state = p.getJointState(self.quadruped,jointIndex=7)[0]
        J8_state = p.getJointState(self.quadruped,jointIndex=8)[0]
        J11_state = p.getJointState(self.quadruped,jointIndex=11)[0]
        J12_state = p.getJointState(self.quadruped,jointIndex=12)[0]
        J13_state = p.getJointState(self.quadruped,jointIndex=13)[0]
        J16_state = p.getJointState(self.quadruped,jointIndex=16)[0]
        J17_state = p.getJointState(self.quadruped,jointIndex=17)[0]
        J18_state = p.getJointState(self.quadruped,jointIndex=18)[0]

        return [J1_state,J2_state,J3_state,J6_state,J7_state,J8_state,J11_state,J12_state,J13_state,J16_state,J17_state,J18_state]

    def get_fitness(self):
        # Compute and return the fitness score based on the performance of the quadruped
        # Implement this based on your specific objectives
        fitness=0.0
        #TODO 
        distance_x= p.getLinkState(self.quadruped,0)[0][0]
        distance_y= p.getLinkState(self.quadruped,0)[0][1]
        ori = p.getLinkState(self.quadruped,0)[1]
        angle_z = p.getEulerFromQuaternion(ori)[2]*180/np.pi

        fitness = np.sqrt(distance_x*distance_x + distance_y*distance_y)

        return fitness 

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
    # # Save the best genome if desired
    # TODO Save the best genome 

# Use the best genome for control
# Load the best genome #TODO
#best_genome = neat.Checkpointer.restore_checkpoint("best_genome.txt")
#best_net = neat.nn.FeedForwardNetwork.create(best_genome, config)

# Quadruped control loop using the best genome
quadruped_env.reset()
quadruped_state = quadruped_env.get_quadruped_state()
while True:
    inputs = quadruped_state  # Replace with appropriate inputs based on the quadruped's state
    outputs = best_net.activate(inputs)
    joint_angles = outputs[:NUM_OUTPUTS]
    quadruped_env.step(joint_angles)
    quadruped_state = quadruped_env.get_quadruped_state()