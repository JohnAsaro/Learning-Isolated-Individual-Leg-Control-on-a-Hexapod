#Note: 

# Script will reset webots world after each run, this is to avoid deterioration of the physics in the webots environment

import threading # To always be taking in keyboard I/O
from controller import Robot, GPS, Supervisor, Keyboard, TouchSensor
import math
import random
import pickle
import os
import csv
import copy

# Fitness Function Used
simple_distance = False # Fitness is just based on distance traveled 
static_stability = False # Fitness is based on distance traveled while keeping the robot statically stable
touch_ground_penalty = True # Penalize the robot for touching the ground with anything that isn't its feet
fitness_functions = [static_stability, simple_distance, touch_ground_penalty] # List of fitness functions

if sum(fitness_functions) > 1:
    raise ValueError("Only one fitness function can be selected at a time.")
elif sum(fitness_functions) == 0:
    raise ValueError("At least one fitness function must be selected.")

# Constants
DISABLED_LEG = 1 # Disable this leg

NUM_LEGS = 6
LEG_PARAMS = 3
POPULATION_SIZE = 50
MUTATION_RATE = 0.1
TIME_STEP = 32
NUM_EVALS = 1 # Evaluate a given individual this many times

# Base joint range of motion
MAX_FOWARD_BEND_BASE = math.radians(90)
MIN_FOWARD_BEND_BASE = math.radians(-90)

# Shoulder range of motion
MAX_FOWARD_BEND_SHOULDER = math.radians(60)
MIN_FOWARD_BEND_SHOULDER = math.radians(-75)

# Knee range of motion
MAX_FORWARD_BEND_KNEE = math.radians(-20)
MIN_FORWARD_BEND_KNEE = math.radians(-120)

# Limit range of motion to + or - this many degrees
LIMIT_ROM = 5

# Initialize Supervisor and Devices
robot = Supervisor()
gps = robot.getDevice("gps")
gps_back = robot.getDevice("gps_back")
gps_front = robot.getDevice("gps_front")
gps.enable(TIME_STEP)
gps_back.enable(TIME_STEP)
gps_front.enable(TIME_STEP)

keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

# Global debug flag
debug_mode = False # Print info about motor positions in real time
print_fitness = True # Print Amplitude, Phase, Offset, and Fitness info
print_stability = False
print_ground = False # Print if the robot is touching the ground or not

# Function to poll the keyboard in a separate thread
def poll_keyboard():
    global debug_mode
    global print_fitness
    global print_stability
    global print_ground
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
        if key in [ord('E'), ord('e')]:  # Toggle stability print statements on 'E' press
            print_stability = not print_stability
            print("\n")
            print("_______________________________________________________\n")
            print(f"Stability print statements {'enabled' if debug_mode else 'disabled'}")
            print("_______________________________________________________\n")
        if key in [ord('R'), ord('r')]:  # Toggle ground print statements on 'R' press
            print_ground = not print_ground
            print("\n")
            print("_______________________________________________________\n")
            print(f"Ground print statements {'enabled' if debug_mode else 'disabled'}")
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

robot_feet = ["RAS", "LAS", "RMS", "LMS", "RPS", "LPS"]
feet_positions = ["ras_gps", "las_gps", "rms_gps", "lms_gps", "rps_gps", "lps_gps"]
for gps_pos in feet_positions:
    foot_gps = robot.getDevice(gps_pos)
    foot_gps.enable(TIME_STEP)

init_positions = [0.699903958534031, 0.7874305232509808, -2.299885916546561,
                -0.7001514218999718, 0.7861381796996287, -2.299962950200891,
                 0.0003427591794476062, 0.7861387560413399, -2.299951670216532,
                -0.00015088237499079072, 0.7861387490342996, -2.299951771152082,
                -0.699818778125061, 0.7861520889739678, -2.2999896140615927,
                0.6995872274087394, 0.7874096205527307, -2.3000106220204892]

touch_sensors = [] # Foot touch sensors

for foot in robot_feet:
    if robot_feet.index(foot) != DISABLED_LEG: # If not disabled leg get in the club
        touch_sensors.append((robot.getDevice(foot), robot_feet.index(foot))) # Tuple of the device and its indexs
        TouchSensor.enable(robot.getDevice(foot), TIME_STEP)
    else: # Otherwise turn off sensor
        TouchSensor.disable(robot.getDevice(foot))
                   
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


def get_feet_touching_ground():
    feet_touching_ground = []
    for sensor in touch_sensors:
        if TouchSensor.getValue(sensor[0]) > 0.0: 
            gps_name = feet_positions[sensor[1]] # Name of the def used to track translation of this sensor
            foot_gps = robot.getDevice(gps_name)
            foot_position = foot_gps.getValues()
            feet_touching_ground.append((sensor[1], foot_position))
    return feet_touching_ground 
    
def convert_to_2d(com, leg_points): 
    # Takes the 3d vector returned by the previous function and converts it to 2d, 
    # purging each leg vector that has a z value higher than the center of mass,
    # this is done to ensure that the center of mass is always above the legs 

    com_x, com_y, com_z = com # Unpack the center of mass coordinates
    leg_positions = [leg[1] for leg in leg_points] # Get the leg positions
    leg_positions = [leg for leg in leg_positions if leg[2] < com_z] # Filter out legs above the COM

    # Project leg points to the x-y plane
    polygon_pts_2d = [(x, y) for x, y, z in leg_positions]

    return (com_x, com_y), polygon_pts_2d

def convex_hull(points): # Get the convex hull of the inputted set of points using Andrew's monotone chain convex hull algorithm

    points = sorted(set(points)) # Sort the points lexicographically (tuples are compared lexicographically)

    if len(points) <= 1: # Base case: if there are 0 or 1 points, the convex hull is the same as the input points
        return points


    def cross(o, a, b): # 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product
        # Returns a positive value, if OAB makes a counter-clockwise turn,
        # negative for clockwise turn, and zero if the points are collinear
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    # Build lower hull 
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # Build upper hull
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Concatenation of the lower and upper hulls gives the convex hull
    # Last point of each list is omitted because it is repeated at the beginning of the other list
    return lower[:-1] + upper[:-1]

def is_statically_stable(polygon_pts, com): # Find if the center of mass is inside a polygon drawn between the point each leg touches the ground
    # Preprocess the points
    com_2d, polygon_pts_2d = convert_to_2d(com, polygon_pts) # Get the 2D points of the legs
    polygon_edges = convex_hull(polygon_pts_2d) # Get the convex hull of the points

    com_x, com_y = com_2d # Unpack the center of mass coordinates
    n = len(polygon_edges) # Number of vertices in the polygon
    if n < 3: # Not enough points to form a polygon
        return False 

    counter = 0
    for i in range(n):
        x1, y1 = polygon_edges[i]
        x2, y2 = polygon_edges[(i + 1) % n] # Next point in the polygon, wrapping around to the first point
        
        if (com_y < y1) != (com_y < y2) and \
        com_x < x1 + ((com_y - y1) / (y2 - y1)) * (x2 - x1):
            counter += 1

    if counter % 2 == 1: # Odd number of intersections means the point is inside the polygon
        return True 
    return False # Even number of intersections means the point is outside the polygon

# Evaluate one leg while other legs use the best individuals
def evaluate_leg(leg_index, individual, best_individuals):
    EVAL_TOTAL = 0

    for _ in range(NUM_EVALS): # Evaluate this individual a number of times
        reset_robot()
        last_positions = {i: init_positions[i] for i in range(NUM_LEGS * LEG_PARAMS)}  # Initialize last positions for each motor
        start_time = robot.getTime()
        max_distance, stability_sum, stability_samples, touched_ground_sum, touched_ground_samples = 0.0, 1.0, 1.0, 1.0, 1.0
        ground_pos = (6.17248, 24.1324, 60.5806) # Initial ground height
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
                            motors[i * 3 + j].setPosition(init_positions[i * 3 + j])            
                        if j == 1:
                            motors[i * 3 + j].setPosition(init_positions[i * 3 + j])            
                        if j == 2:
                            motors[i * 3 + j].setPosition(init_positions[i * 3 + j])
                     
            robot.step(TIME_STEP)
            
            # Distance function
            body_pos = gps.getValues()
            distance = max(body_pos[0] - initial_pos[0], 0) # Calculate distance traveled foward in the x direction
            max_distance = max(max_distance, distance)

            # Stability function
            foot_info = get_feet_touching_ground()
            stable = is_statically_stable(foot_info, body_pos) # Check if the robot is statically stable
            if stable: # Check if the robot is statically stable
                stability_sum += 1.0
            stability_samples += 1.0
            if print_stability:
                print(f"Feet touching ground: {foot_info}, we are {'stable' if stable else 'not stable'}")
    
            # Touch ground penalty function
            z_range_front, z_range_mid, z_range_back = 0.125, 0.125, 0.125 # Z, Y, X range of the body
            z_front, z_mid, z_back = gps_front.getValues()[2], body_pos[2], gps_back.getValues()[2]
            ground_info = []
            for foot in foot_info:
                ground_info.append(foot[1][2]) # Get z-pos of ground
            
            flipped = False
            if len(ground_info) > 1:
                ground_pos = min(ground_info) 
            elif len(ground_info) == 1:
                ground_pos = ground_info[0]
            else:
                flipped = True # If no legs touching the ground, we have flipped

            if flipped or z_front + z_range_front > ground_pos > z_front - z_range_front or z_mid + z_range_mid > ground_pos > z_mid - z_range_mid or z_back + z_range_back > ground_pos > z_back - z_range_back: # If the center of mass is within some range of the ground height
                touched_ground_sum += 1.0
            touched_ground_samples += 1.0
            if print_ground:
                print(f"We are {'touching the ground' if flipped or z_front + z_range_front > ground_pos > z_front - z_range_front or z_mid + z_range_mid > ground_pos > z_mid - z_range_mid or z_back + z_range_back > ground_pos > z_back - z_range_back else 'not touching the ground'}")
                print(f"Ground pos: {ground_pos}, z pos front: {z_front}, z pos mid: {z_mid}, z pos back: {z_back}")

        stability_multiplier = stability_sum / stability_samples # Percentage of the time the robot was stable applied as a multiplier
        touched_ground_multiplier = touched_ground_sum / touched_ground_samples # Percentage of the time the robot was touching the ground applied as a multiplier

        THIS_EVAL = 0.0 # Fitness value   
        if simple_distance:
            THIS_EVAL = max_distance # Fitness function based solely on distance traveled
        elif static_stability:
            THIS_EVAL = max_distance * stability_multiplier # Fitness function based on distance traveled while keeping the robot statically stable
        elif touch_ground_penalty:
            THIS_EVAL = max_distance * touched_ground_multiplier # Fitness function based on distance traveled while keeping body off the ground
        
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
def save_state(filename, populations, best_individuals, best_overall, generation, effective_generation):
    with open(filename, 'wb') as file:
        pickle.dump({'populations': populations, 
                     'best_individuals': best_individuals, 
                     'best_overall': best_overall,
                     'generation': generation,
                     'effective_generation': effective_generation}, file)
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
    ext = ""  # No extension since checkpoints don't have one
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
    return state['populations'], state['best_individuals'], state['best_overall'], state['generation'], state['effective_generation']

best_fitnesses_file, n1 = get_next_best_fitnesses_file()
gens_per_run = 1 # 1 generations per run to avoid deterioration 

load_checkpoint, n2 = get_latest_checkpoint() 
# load_checkpoint = None # Set to None if starting fresh

# Initialize state
if load_checkpoint:
    try:
        populations, best_individuals, best_overall, generation, effective_generation = load_state(load_checkpoint)
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
    effective_generation = -1 # Effective generation, since we are are doing a full generation for each leg, this counts the amount of generations done on each leg combined

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
        csv_writer.writerow(['Run', 'Generation', 'Effective Generation', 'Leg Index', 'Individual Index',
                             'Fitness', 'Amplitude', 'Phase', 'Offset'])

with open(csv_file_name, mode='a', newline='') as csv_file:

    csv_writer = csv.writer(csv_file)

    while gens_per_run > 0:
        print(f"Generation {generation}")

        for leg_index in range(NUM_LEGS):
            if leg_index != DISABLED_LEG:
                effective_generation += 1
                print(f"Effective Generation {effective_generation}")
            
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
                        effective_generation,  # Effective generation
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

        best_overall = copy.deepcopy(best_overall) # Save deep copy of best overall
        best_individuals = copy.deepcopy(best_individuals) # Save deep copy of best individuals

        for leg_index, population in enumerate(populations):
            for individual_index, individual in enumerate(population):
                individual['fitness'] = 0.0
               
        save_state(checkpoint_file, populations, best_individuals, best_overall, generation, effective_generation) # Make a checkpoint
        
        generation += 1
        gens_per_run = gens_per_run - 1

robot.worldReload()