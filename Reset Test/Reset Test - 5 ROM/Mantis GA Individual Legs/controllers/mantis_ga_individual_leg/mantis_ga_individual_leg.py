#Note: 

# Script will reset webots world after each run, this is to avoid deterioration of the physics in the webots environment

import threading # To always be taking in keyboard I/O
from controller import Robot, GPS, Supervisor, Keyboard
import math
import random
import pickle
import os
import csv


# Constants
DISABLED_LEG = -1 # Disable this leg
STUCK_BASE = math.radians(0)
STUCK_SHOULDER = math.radians(0)
STUCK_KNEE = math.radians(-30)

NUM_LEGS = 6
LEG_PARAMS = 3
POPULATION_SIZE = 50
MUTATION_RATE = 0.1
TIME_STEP = 32
HEIGHT_WEIGHT = 0.5
NUM_EVALS = 1 # Evaluate a given individual this many times

# Base joint range of motion
MAX_FOWARD_BEND_BASE = math.radians(90)
MIN_FOWARD_BEND_BASE = math.radians(-90)

# Shoulder range of motion
MAX_FOWARD_BEND_SHOULDER = math.radians(90)
MIN_FOWARD_BEND_SHOULDER = math.radians(-90)

# Knee range of motion
MAX_FORWARD_BEND_KNEE = math.radians(-20)
MIN_FORWARD_BEND_KNEE = math.radians(-120)

# Limit range of motion to + or - this many degrees
LIMIT_ROM = 5

# Initialize Supervisor and Devices
robot = Supervisor()
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

# Global debug flag
debug_mode = False # Print info about motor positions in real time
print_fitness = True # Print Amplitude, Phase, Offset, and Fitness info for each leg

# Function to poll the keyboard in a separate thread
def poll_keyboard():
    global debug_mode
    global print_fitness
    while True:
        key = keyboard.getKey()
        if key in [ord('Q'), ord('q')]:  # Toggle debug mode on 'Q' key press
            debug_mode = not debug_mode
            print("\n")
            print("_______________________________________________________\n")
            print(f"Debug mode {'enabled' if debug_mode else 'disabled'}")
            print("_______________________________________________________\n")
        if key in [ord('W'), ord('w')]:  # Toggle individual fitness print statements on 'W' press
            print_fitness = not print_fitness
            print("\n")
            print("_______________________________________________________\n")
            print(f"Individual fitness print statements {'enabled' if debug_mode else 'disabled'}")
            print("_______________________________________________________\n")

# Start the keyboard polling thread
keyboard_thread = threading.Thread(target=poll_keyboard, daemon=True)
keyboard_thread.start()

# Motor Initialization
#
# Meaning of the motor charcters:
# - 'R': Right / 'L': Left
# - 'A': Front / 'M': Middle / 'P': Rear
# - 'C': Base / 'F': Shoulder / 'T': Knee

motor_names =  ["RAC", "RAF", "RAT", 
                "LAC", "LAF", "LAT",
               "RMC", "RMF", "RMT",
               "LMC", "LMF", "LMT", 
               "RPC", "RPF", "RPT",
               "LPC", "LPF", "LPT"]

motors = [robot.getDevice(name) for name in motor_names]

init_positions = [0.699903958534031, 0.7874305232509808, -2.299885916546561,
                 -0.7001514218999718, 0.7861381796996287, -2.299962950200891,
                 0.0003427591794476062, 0.7861387560413399, -2.299951670216532,
                -0.00015088237499079072, 0.7861387490342996, -2.299951771152082,
                -0.699818778125061, 0.7861520889739678, -2.2999896140615927,
                0.6995872274087394, 0.7874096205527307, -2.3000106220204892]

for motor, init_pos in zip(motors, init_positions):
    #motor.setPosition(0.0)
    motor.setPosition(init_pos)


# Set initial position and rotation for reset
initial_position = [6.160960988497287, 24.08646487667829, 61.3591783745818]  # Hardcoded position [x, y, z]
initial_rotation = [0.03125814128046004, -0.03913732036488227, 0.998744811630252, 1.3035737695105172]  # Hardcoded rotation [axis_x, axis_y, axis_z, angle]


root = robot.getRoot()
children_field = root.getField("children")
robot_node = next((children_field.getMFNode(i) for i in range(children_field.getCount())
                   if children_field.getMFNode(i).getTypeName() == "Mantis"), None)
translation_field = robot_node.getField("translation")
rotation_field = robot_node.getField("rotation")
translation_field.setSFVec3f(initial_position) 
rotation_field.setSFRotation(initial_rotation)

# Generate a random individual
def create_individual():
    return {
        "amplitude": [random.uniform(0, 0.5) for _ in range(LEG_PARAMS)],
        "phase": [random.uniform(0, 2 * math.pi) for _ in range(LEG_PARAMS)],
        "offset": [random.uniform(-0.5, 0.5) for _ in range(LEG_PARAMS)],
        "fitness": 0.0
    }

# Clamp values within a range
def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)
    
# Reset robot to initial state
def reset_robot():
    for motor, init_pos in zip(motors, init_positions):
        #motor.setPosition(0.0)
        motor.setPosition(init_pos)
    translation_field.setSFVec3f(initial_position)
    rotation_field.setSFRotation(initial_rotation)
    for _ in range(10):
        robot.step(TIME_STEP)

# Evaluate one leg while other legs use the best individuals
def evaluate_leg(leg_index, individual, best_individuals):
    EVAL_TOTAL = 0

    for _ in range(NUM_EVALS): # Evaluate this individual a number of times
        reset_robot()
        last_positions = {i: init_positions[i] for i in range(NUM_LEGS * LEG_PARAMS)}  # Initialize last positions for each motor
        start_time = robot.getTime()
        max_distance, height_sum, height_samples = 0.0, 0.0, 0
        initial_pos = gps.getValues()
        f = 0.5  # Gait frequency
    
        if leg_index == DISABLED_LEG:
            return
    
        while robot.getTime() - start_time < 20.0:
            time = robot.getTime()
            for i in range(NUM_LEGS):
                if i != DISABLED_LEG:
                    leg_params = individual if i == leg_index else best_individuals[i] # For this leg evaluate the individual, and deafult all other legs to their best individuals
                    for j in range(LEG_PARAMS):
                        position = (leg_params["amplitude"][j] *
                                    math.sin(2.0 * math.pi * f * time + leg_params["phase"][j]) +
                                    leg_params["offset"][j])
                        
                        if debug_mode:
                            print(f"Before we limit movement, we attempt to go from {last_positions[i * 3 + j]} to {position}")
                        # Clamp the position change to +/- LIMIT_ROM degrees, as motors can only move so much
                        position = clamp(position, last_positions[i * 3 + j] - math.radians(LIMIT_ROM), last_positions[i * 3 + j] + math.radians(LIMIT_ROM)) 
                        if debug_mode:
                            print(f"After we limit movement, we attempt to go from {last_positions[i * 3 + j]} to {position}")
                        
                       
                        if j == 0:
                            if debug_mode:
                                print(f"First joint (base joint) position, before clamp: {position}") # Print position of base joint
                            position = clamp(position, MIN_FOWARD_BEND_BASE, MAX_FOWARD_BEND_BASE)
                            if debug_mode:
                                print(f"First joint (base joint) position, after clamp: {position}") # Print position of base joint
                        if j == 1:
                            if debug_mode:
                                print(f"Second joint (shoulder joint) position, before clamp: {position}") # Print position of shoulder joint
                            position = clamp(position, MIN_FOWARD_BEND_SHOULDER, MAX_FOWARD_BEND_SHOULDER)
                            if debug_mode:
                                print(f"Second joint (shoulder joint) position, after clamp: {position}") # Print position of shoulder joint
                        if j == 2:  # Clamp knee joint
                            if debug_mode:
                                print(f"Third joint (knee joint) position, before clamp: {position}") # Knee joint before clamp
                            position = clamp(position, MIN_FORWARD_BEND_KNEE, MAX_FORWARD_BEND_KNEE)
                            if debug_mode:
                                print(f"Third joint (knee joint) position, after clamp: {position}") # Knee joint after clamp
                     
                        last_positions[i * 3 + j] = position
                        motors[i * 3 + j].setPosition(position)
                else:
                    for j in range(LEG_PARAMS):
                        if j == 0:
                            motors[i * 3 + j].setPosition(STUCK_BASE)            
                        if j == 1:
                            motors[i * 3 + j].setPosition(STUCK_SHOULDER)            
                        if j == 2:
                            motors[i * 3 + j].setPosition(STUCK_KNEE)
                     
            robot.step(TIME_STEP)
            current_pos = gps.getValues()
            distance = math.sqrt((current_pos[0] - initial_pos[0]) ** 2 + (current_pos[2] - initial_pos[2]) ** 2)
            max_distance = max(max_distance, distance)
            height_sum += current_pos[1]
            height_samples += 1
    
        avg_height = height_sum / height_samples if height_samples > 0 else 0.0
        THIS_EVAL = max_distance + avg_height * HEIGHT_WEIGHT
        EVAL_TOTAL = THIS_EVAL + EVAL_TOTAL

    individual["fitness"] = EVAL_TOTAL/NUM_EVALS # Average of total evals
 
    # Print all info if debug mode on
    if print_fitness:
        print("\n--- Info for Leg", leg_index, "---")
        print("Amplitude:", individual["amplitude"])
        print("Phase:", individual["phase"])
        print("Offset:", individual["offset"])
        print("Fitness:", individual["fitness"])
        print("--------------------------------------\n")
# Mutation
def mutate(individual):
    for i in range(LEG_PARAMS):
        if random.random() < MUTATION_RATE:
            individual["amplitude"][i] += random.uniform(-0.05, 0.05)
            individual["phase"][i] += random.uniform(-0.05, 0.05)
            individual["offset"][i] += random.uniform(-0.05, 0.05)

# Crossover
def crossover(parent1, parent2):
    child = create_individual()
    for i in range(LEG_PARAMS):
        if random.choice([True, False]):
            child["amplitude"][i] = parent1["amplitude"][i]
            child["phase"][i] = parent1["phase"][i]
            child["offset"][i] = parent1["offset"][i]
        else:
            child["amplitude"][i] = parent2["amplitude"][i]
            child["phase"][i] = parent2["phase"][i]
            child["offset"][i] = parent2["offset"][i]
    mutate(child)
    return child

# Select a parent
def select_parent(population):
    total_fitness = sum(ind["fitness"] for ind in population)
    if total_fitness == 0:
        return random.choice(population)
    selection_probs = [ind["fitness"] / total_fitness for ind in population]
    return random.choices(population, weights=selection_probs, k=1)[0]

# Evolve a population
def evolve_population(population):
    population.sort(key=lambda ind: ind["fitness"], reverse=True)
    new_population = population[:POPULATION_SIZE // 2]
    while len(new_population) < POPULATION_SIZE:
        parent1 = select_parent(population)
        parent2 = select_parent(population)
        child = crossover(parent1, parent2)
        new_population.append(child)
    return new_population
   
# Function to generate the next best fitnesses file
def get_next_best_fitnesses_file():
    base_directory = "best_fitnesses"
    base_name = "best_fitnesses_run_"
    ext = ".txt"
    n1 = 1
    
    # Ensure the directory exists
    if not os.path.exists(base_directory):
        os.makedirs(base_directory)
    
    # Generate file name with incremental number
    while os.path.exists(os.path.join(base_directory, f"{base_name}{n1}{ext}")):
        n1 += 1
    return (os.path.join(base_directory, f"{base_name}{n1}{ext}")), n1

# Save state to file
def save_state(filename, populations, best_individuals, best_overall, generation):
    with open(filename, 'wb') as file:
        pickle.dump({'populations': populations, 
                     'best_individuals': best_individuals, 
                     'best_overall': best_overall,
                     'generation': generation}, file)
    print(f"State saved to {filename}")
 
# Main Evolution Loop

# Define the directory for saves
saves_directory = "saves"

# Check if the saves directory exists, and create it if it does not
if not os.path.exists(saves_directory):
    os.makedirs(saves_directory)

# Define the directory for saves
saves_directory = "saves"

# Check if the saves directory exists, and create it if it does not
if not os.path.exists(saves_directory):
    os.makedirs(saves_directory)

def get_latest_checkpoint():
    saves_directory = "saves"  # Define the directory where saves are stored
    base_name = "run_"
    gen_base = "_generation"
    ext = ""  # No extension since your checkpoints don't have one
    latest_n = 0
    latest_m = 0
    latest_checkpoint = None

    # Ensure the directory exists to avoid FileNotFoundError
    if not os.path.exists(saves_directory):
        print("No saves directory found.")
        return None

    # Iterate through files in the saves directory
    for filename in os.listdir(saves_directory):
        if filename.startswith(base_name) and gen_base in filename:
            try:
                # Extract n and m from the filename
                n_part = filename[len(base_name):filename.index(gen_base)]
                m_part = filename[filename.index(gen_base) + len(gen_base):]
                n = int(n_part)
                m = int(m_part)

                # Update if this file is more recent
                if n > latest_n or (n == latest_n and m > latest_m):
                    latest_n = n
                    latest_m = m
                    latest_checkpoint = os.path.join(saves_directory, filename)  # Include the path to the file

            except ValueError:
                # Ignore files that don't match the expected pattern
                continue

    print(f"Latest checkpoint: {latest_checkpoint}")
    return latest_checkpoint, latest_n

# Load state from file
def load_state(filename):
    with open(filename, 'rb') as file:
        state = pickle.load(file)
    print(f"State loaded from {filename}")
    return state['populations'], state['best_individuals'], state['best_overall'], state['generation']

best_fitnesses_file, n1 = get_next_best_fitnesses_file()
gens_per_run = 1 # 1 generations per run to avoid deterioration 

load_checkpoint, n2 = get_latest_checkpoint() 
# load_checkpoint = None # Set to None if starting fresh

# Initialize state
if load_checkpoint:
    try:
        populations, best_individuals, best_overall, generation = load_state(load_checkpoint)
        generation = generation + 1
        print(f"Loaded state from {load_checkpoint}: Resuming from generation {generation}")
    except FileNotFoundError:
        print(f"Error: File '{load_checkpoint}' not found.")
        exit(1)
    except Exception as e:
        print(f"Error loading file '{load_checkpoint}': {e}")
        exit(1)
else:
    populations = [[create_individual() for _ in range(POPULATION_SIZE)] for _ in range(NUM_LEGS)]
    best_individuals = [create_individual() for _ in range(NUM_LEGS)] # Initial random best individuals (So robot can walk)
    best_overall = best_individuals
    generation = 0  # First gen

checkpoint_file = os.path.join(saves_directory, f"run_{n2+1}_generation{generation}") # Checkpoint organized by run and generation

# Define the directory for the data
data_directory = "data"
csv_file_name = os.path.join(data_directory, "evolution_data.csv")

print(f"Progress will be saved to: {checkpoint_file}, data will be saved to: {csv_file_name}")

# Check if the directory exists, and create it if it does not
if not os.path.exists(data_directory):
    os.makedirs(data_directory)

# Ensure CSV file has a header if it doesn't exist
if not os.path.exists(csv_file_name):
    with open(csv_file_name, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        # Write the header row
        csv_writer.writerow(['Run', 'Generation', 'Leg Index', 'Individual Index',
                             'Fitness', 'Amplitude', 'Phase', 'Offset'])

with open(csv_file_name, mode='a', newline='') as csv_file:

    csv_writer = csv.writer(csv_file)

    while gens_per_run > 0:
        print(f"Generation {generation}")
        
        for leg_index in range(NUM_LEGS):
            
            if leg_index != DISABLED_LEG:
                print(f"Evaluating leg {leg_index} for generation {generation}")
            
            for individual_index, individual in enumerate(populations[leg_index]): # Evaluate individual
                if print_fitness == True and leg_index != DISABLED_LEG:
                    print(f"Evaluating individual {individual_index} for leg {leg_index} in generation {generation}")
                
                if leg_index != DISABLED_LEG:
                    evaluate_leg(leg_index, individual, best_individuals)            
                
                    # Log individual fitness/info to file
                    csv_writer.writerow([
                        n2+1,  # Run number
                        generation,  # Current generation
                        leg_index,  # Leg index
                        individual_index,  # Individual index
                        individual['fitness'],  # Fitness
                        individual['amplitude'],  # Amplitude list
                        individual['phase'],  # Phase list
                        individual['offset']  # Offset list
                    ])
                        
                # Update the best individual for this leg
                if leg_index != DISABLED_LEG:
                    best_individuals[leg_index] = max(populations[leg_index], key=lambda ind: ind["fitness"])
                    if best_individuals[leg_index]['fitness'] > best_overall[leg_index]['fitness']:
                        best_overall[leg_index] = best_individuals[leg_index]    
                                
        # Evolve each population
        for leg_index in range(NUM_LEGS):
            if leg_index != DISABLED_LEG:
                populations[leg_index] = evolve_population(populations[leg_index])

            # Write to evolution file and checkporint
            with open(best_fitnesses_file, "w") as file:
                for leg_index, best in enumerate(best_individuals):
                    if leg_index != DISABLED_LEG:
                        print(f"\n--- Best Individual for Leg {leg_index} ---")
                        print(f"Fitness: {best['fitness']:.3f}")
                        print(f"Amplitude: {best['amplitude']}")
                        print(f"Phase: {best['phase']}")
                        print(f"Offset: {best['offset']}")
                        print("------------------------------------------")
    
                        file.write(f"Generation {generation}, Leg {leg_index}, Best Fitness: {best['fitness']:.3f}, "
                                   f"Amplitude: {best['amplitude']}, "
                                   f"Phase: {best['phase']}, "
                                   f"Offset: {best['offset']}\n"
                                   f"-------------------------------------\n"
                                   f"Generation {generation}, Leg {leg_index}, Best Overall: {best_overall[leg_index]['fitness']:.3f}, "
                                   f"Amplitude: {best_overall[leg_index]['amplitude']}, "
                                   f"Phase: {best_overall[leg_index]['phase']}, "
                                   f"Offset: {best_overall[leg_index]['offset']}\n"
                                   f"-------------------------------------\n")
                        file.flush()
       
        save_state(checkpoint_file, populations, best_individuals, best_overall, generation) # Make a checkpoint
        
        generation += 1
        gens_per_run = gens_per_run - 1

# Reset the environment after completing the run
print("Resetting Webots environment...")
robot.worldReload()
