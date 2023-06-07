import pybullet as p
import pybullet_data
import time
import neat
import random
# Quadruped simulation parameters
QUADRUPED_URDF_PATH = "../anymal/urdf/anymal.urdf"
NUM_LEGS = 1
LEG_JOINT_NAMES = ["LF_HAA", "LF_HFE", "LF_KFE"]  # Replace with actual joint names
LEG_JOINT_NUMBERS = [1,2,3]

MAX_JOINT_FORCE = 80  # Maximum joint force applied by the motors
NUM_STEPS = 10
# NEAT parameters
NUM_INPUTS = 3  # Replace with the number of inputs based on your quadruped's state
#NUM_OUTPUTS = NUM_LEGS * len(LEG_JOINT_NAMES)  # Each leg has joint control
NUM_OUTPUTS = 3  # Each leg has joint control
# NEAT training loop
NUM_GENERATIONS = 2
# Quadruped environment class
class QuadrupedEnv:
    def __init__(self):
        p.connect(p.GUI)  # or p.DIRECT for headless mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")  # Load the ground plane
        self.quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition=[0, 0, 0.8], useFixedBase = True) #Fixed
        p.setGravity(0, 0, -9.81)
        self.joint_ids = []
        # for leg_id in range(NUM_LEGS):
        #     for joint_name in LEG_JOINT_NAMES:
        #         joint_id = p.getJointInfo(self.quadruped, leg_id, joint_name)
        #         self.joint_ids.append(joint_id[0])
        self.joint_ids = LEG_JOINT_NUMBERS

    def reset(self):
        # Reset the simulation
        p.setGravity(0, 0, -9.81)
        #TODO modify to call every joint
        p.resetJointState(self.quadruped,jointIndex=1,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=2,targetValue=0)
        p.resetJointState(self.quadruped,jointIndex=3,targetValue=0)

    def step(self, joint_angles=[0,0,0]):
        #TODO modify to call every joint
        p.setJointMotorControl2(self.quadruped,jointIndex=1,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[0],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=2,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[1],force=MAX_JOINT_FORCE)
        p.setJointMotorControl2(self.quadruped,jointIndex=3,controlMode=p.POSITION_CONTROL, targetPosition=joint_angles[2],force=MAX_JOINT_FORCE)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)  # Control the simulation speed

    def get_quadruped_state(self):
        # Retrieve and return the state of the quadruped (e.g., position, orientation, velocities)
        # Implement this based on your specific needs

        #TODO 

        return [random.random(),random.random(),random.random()]

    def get_fitness(self):
        # Compute and return the fitness score based on the performance of the quadruped
        # Implement this based on your specific objectives
        
        #TODO 

        return -1

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