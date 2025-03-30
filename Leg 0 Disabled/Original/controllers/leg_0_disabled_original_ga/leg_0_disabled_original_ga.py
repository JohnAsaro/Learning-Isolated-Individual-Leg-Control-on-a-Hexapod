#Note: 

# Script will reset webots world after each run, this is to avoid deterioration of the physics in the webots environment

import threading # To always be taking in keyboard I/O
from controller import Robot, GPS, Supervisor, Keyboard
import math
import random
import pickle
import os
import csv
import copy

# Constants
DISABLED_LEG = 0 # Disable this leg

PARAMS = 18
POPULATION_SIZE = 50
MUTATION_RATE = 0.1
TIME_STEP = 32
HEIGHT_WEIGHT = 0.2
NUM_EVALS = 1 # Evaluate any given individual 3 times

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
print_fitness = True # Print Amplitude, Phase, Offset, and Fitness info

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

init_positions = [0.699903958534031, 0.7874305232509808, -2.299885916546561,
                -0.7001514218999718, 0.7861381796996287, -2.299962950200891,
                 0.0003427591794476062, 0.7861387560413399, -2.299951670216532,
                -0.00015088237499079072, 0.7861387490342996, -2.299951771152082,
                -0.699818778125061, 0.7861520889739678, -2.2999896140615927,
                0.6995872274087394, 0.7874096205527307, -2.3000106220204892]
                   
if DISABLED_LEG == 0:
    init_positions[2] = 0.999885916546561
if DISABLED_LEG == 1:
    init_positions[5] = 0.999962950200891
if DISABLED_LEG == 2:
    init_positions[8] = 0.999951670216532
if DISABLED_LEG == 3:
    init_positions[11] = 0.999951771152082
if DISABLED_LEG == 4:
    init_positions[14] = 0.9999896140615927
if DISABLED_LEG == 5:
    init_positions[17] = 1.0000106220204892

motors = [robot.getDevice(name) for name in motor_names]

for motor, init_pos in zip(motors, init_positions):
    motor.setPosition(init_pos)
    #motor.setPosition(0.0)


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
        "amplitude": [random.uniform(0, 0.5) for _ in range(PARAMS)],
        "phase": [random.uniform(0, 2 * math.pi) for _ in range(PARAMS)],
        "offset": [random.uniform(-0.5, 0.5) for _ in range(PARAMS)],
        "fitness": 0.0
    }

# Clamp values within a range
def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)
    
# Reset robot to initial state
def reset_robot():
    for motor, init_pos in zip(motors, init_positions):
        motor.setPosition(init_pos)
        #motor.setPosition(0.0)
    translation_field.setSFVec3f(initial_position)
    rotation_field.setSFRotation(initial_rotation)
    for _ in range(10):
        robot.step(TIME_STEP)

# Evaluate fitness of an individual
def evaluate(individual):
    EVAL_TOTAL = 0

    for _ in range(NUM_EVALS): # Evaluate this individual a number of times
        reset_robot()
        last_positions = {i: init_positions[i] for i in range(PARAMS)}  # Initialize last positions for each motor
        start_time = robot.getTime()
        max_distance, height_sum, height_samples = 0.0, 0.0, 0
        initial_pos = gps.getValues()
        f = 0.5  # Gait frequency

        # Calculate the range of indices for the disabled leg
        disabled_leg_start = DISABLED_LEG * 3
        disabled_leg_end = disabled_leg_start + 3
        
        while robot.getTime() - start_time < 20.0:
        
            time = robot.getTime()
            for i in range(PARAMS):
                if disabled_leg_start <= i < disabled_leg_end: # Skip disabled leg
                        if i % 3 == 0:
                            motors[disabled_leg_start].setPosition(init_positions[i])            
                        if i % 3 == 1:
                            motors[disabled_leg_start+1].setPosition(init_positions[i])            
                        if i % 3 == 2:
                            motors[disabled_leg_start+2].setPosition(init_positions[i]) 
                else:
                    position = (individual["amplitude"][i] * math.sin(2.0 * math.pi * f * time + individual["phase"][i])
                            + individual["offset"][i])
                
                    if debug_mode:
                        print(f"Before we limit movement, we attempt to go from {last_positions[i]} to {position}")
                    # Clamp the position change to +/- LIMIT_ROM degrees, as motors can only move so much                    
                    position = clamp(position, last_positions[i] - math.radians(LIMIT_ROM), last_positions[i] + math.radians(LIMIT_ROM))
                    if debug_mode:
                        print(f"After we limit movement, we attempt to go from {last_positions[i]} to {position}")
                        
                    if i % 3 == 0:
                        if debug_mode:
                            print(f"First joint (base joint) position, before clamp: {position}") # Print position of base joint
                        position = clamp(position, MIN_FOWARD_BEND_BASE, MAX_FOWARD_BEND_BASE)
                        if debug_mode:
                            print(f"First joint (base joint) position, after clamp: {position}") # Print position of base joint
                    if i % 3 == 1:
                        if debug_mode:
                            print(f"Second joint (shoulder joint) position, before clamp: {position}") # Print position of shoulder joint
                        position = clamp(position, MIN_FOWARD_BEND_SHOULDER, MAX_FOWARD_BEND_SHOULDER)
                        if debug_mode:
                            print(f"Second joint (shoulder joint) position, after clamp: {position}") # Print position of shoulder joint
                    if i % 3 == 2:  # Clamp knee joint
                        if debug_mode:
                            print(f"Third joint (knee joint) position, before clamp: {position}") # Knee joint before clamp
                        position = clamp(position, MIN_FORWARD_BEND_KNEE, MAX_FORWARD_BEND_KNEE)
                        if debug_mode:
                            print(f"Third joint (knee joint) position, after clamp: {position}") # Knee joint after clamp
                    
                    last_positions[i] = position # Save last position to limit range of motion        
                    motors[i].setPosition(position)
            
            robot.step(TIME_STEP)
            current_pos = gps.getValues()
            distance = math.sqrt((current_pos[0] - initial_pos[0]) ** 2 + (current_pos[1] - initial_pos[1]) ** 2)
            max_distance = max(max_distance, distance)
            height_sum += current_pos[2]
            height_samples += 1
    
        avg_height = height_sum / height_samples if height_samples > 0 else 0.0
        THIS_EVAL = max_distance + avg_height * HEIGHT_WEIGHT
        EVAL_TOTAL = THIS_EVAL + EVAL_TOTAL
        
    individual["fitness"] = EVAL_TOTAL/NUM_EVALS # Average of total evals
    
    # Print all info if debug mode on
    if print_fitness:
        print("\n--------------------------------------")
        print("Amplitude:", individual["amplitude"])
        print("Phase:", individual["phase"])
        print("Offset:", individual["offset"])
        print("Fitness:", individual["fitness"])
        print("--------------------------------------\n")

# Mutation
def mutate(individual):
    for i in range(PARAMS):
        if random.random() < MUTATION_RATE:
            individual["amplitude"][i] += random.uniform(-0.05, 0.05)
            individual["phase"][i] += random.uniform(-0.05, 0.05)
            individual["offset"][i] += random.uniform(-0.05, 0.05)

# Crossover
def crossover(parent1, parent2):
    child = create_individual()
    for i in range(PARAMS):
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
def save_state(filename, populations, best_individual, best_overall, generation):
    with open(filename, 'wb') as file:
        pickle.dump({'populations': populations, 
                     'best_individual': best_individual, # Best individual of this generation
                     'best_overall': best_overall, # Best individual of all generations
                     'generation': generation}, file)
    print(f"State saved to {filename}")
 
def get_latest_checkpoint():
    saves_directory = os.path.join(os.getcwd(), "saves")  # Define the directory where saves are stored
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

# Load state from file
def load_state(filename):
    with open(filename, 'rb') as file:
        state = pickle.load(file)
    print(f"State loaded from {filename}")
    return state['populations'], state['best_individual'], state['best_overall'], state['generation']

best_fitnesses_file, n1 = get_next_best_fitnesses_file()
gens_per_run = 1 # 1 generations per run to avoid deterioration 

load_checkpoint, n2 = get_latest_checkpoint()  # Set to None if starting fresh
#load_checkpoint = None 

# Initialize state
if load_checkpoint:
    try:
        population, best_individual, best_overall, generation = load_state(load_checkpoint)
        generation = generation + 1
        print(f"Loaded state from {load_checkpoint}: Resuming from generation {generation}")
    except FileNotFoundError:
        print(f"Error: File '{load_checkpoint}' not found.")
        exit(1)
    except Exception as e:
        print(f"Error loading file '{load_checkpoint}': {e}")
        exit(1)
else:
    population = [create_individual() for _ in range(POPULATION_SIZE)]
    best_individual = create_individual() # Initial random best individual
    best_overall = best_individual
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
        csv_writer.writerow(['Run', 'Generation', 'Individual Index', 
                             'Fitness', 'Amplitude', 'Phase', 'Offset'])

with open(csv_file_name, mode='a', newline='') as csv_file:
    while gens_per_run > 0:
        print(f"Generation {generation}")

        csv_writer = csv.writer(csv_file)
        # Evaluate each leg independently
        for i, individual in enumerate(population):
            # Evaluate individual
            if print_fitness == True:
                print(f"Evaluating individual {i} in generation {generation}")
            evaluate(individual)
            # Log individual fitness/info to file
            csv_writer.writerow([
                n2+1,  # Run number
                generation,  # Current generation
                i,  # Individual index
                individual['fitness'],  # Fitness
                individual['amplitude'],  # Amplitude list
                individual['phase'],  # Phase list
                individual['offset']  # Offset list
            ])
    
        # Update the best individual for this generation
        best_individual = max(population, key=lambda ind: ind["fitness"])
        best_overall = max(best_individual, best_overall, key=lambda ind: ind["fitness"])
        
        # Evolve each population
        population = evolve_population(population)
        
        with open(best_fitnesses_file, "w") as file:

            # Log the best fitness for each leg
            file.write(f"Generation {generation}, Best Fitness: {best_individual['fitness']:.3f}, "
                       f"Amplitude: {best_individual['amplitude']}, "
                       f"Phase: {best_individual['phase']}, "
                       f"Offset: {best_individual['offset']}\n"
                       f"-------------------------------------\n"
                       f"Generation {generation}, Best Overall: {best_overall['fitness']:.3f}, "
                       f"Amplitude: {best_overall['amplitude']}, "
                       f"Phase: {best_overall['phase']}, "
                       f"Offset: {best_overall['offset']}\n"
                       f"-------------------------------------\n")
                                                        
            # Flush the file to ensure data is written
            file.flush()
            
        if print_fitness == False: 
            print(f"Best Fitness: {best_individual['fitness']:.3f}")
        else:
            print(f"\n--- Best Individual for generation {generation} ---")
            print(f"Fitness: {best_individual['fitness']:.3f}")
            print(f"Amplitude: {best_individual['amplitude']}")
            print(f"Phase: {best_individual['phase']}")
            print(f"Offset: {best_individual['offset']}")
            print("------------------------------------------")
       
        best_overall = copy.deepcopy(best_overall) # Save deep copy of best overall
        for individual_index, individual in enumerate(population):
            individual['fitness'] = 0.0
    
        save_state(checkpoint_file, population, best_individual, best_overall, generation) # Make a checkpoint
        
        generation += 1
        gens_per_run = gens_per_run - 1

# Reset the environment after completing the run
print("Resetting Webots environment...")
robot.worldReload()
