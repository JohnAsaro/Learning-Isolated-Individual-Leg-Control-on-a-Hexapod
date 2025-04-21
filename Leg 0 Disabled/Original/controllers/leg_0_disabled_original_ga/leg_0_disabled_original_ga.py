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

# Constants
DISABLED_LEG = 0 # Disable this leg

PARAMS = 18
POPULATION_SIZE = 50
MUTATION_RATE = 0.1
TIME_STEP = 32
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
print_stability = False

# Function to poll the keyboard in a separate thread
def poll_keyboard():
    global debug_mode
    global print_fitness
    global print_stability
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
            
# Evaluate fitness of an individual
def evaluate(individual):
    EVAL_TOTAL = 0
    
    for _ in range(NUM_EVALS): # Evaluate this individual a number of times
        reset_robot()
        last_positions = {i: init_positions[i] for i in range(PARAMS)}  # Initialize last positions for each motor
        start_time = robot.getTime()
        max_distance, stability_sum, stability_samples = 0.0, 1.0, 1.0
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
            body_pos = gps.getValues()
            foot_info = get_feet_touching_ground()
            distance = max(body_pos[0] - initial_pos[0], 0) # Calculate distance traveled foward in the x direction
            max_distance = max(max_distance, distance)
            stable = is_statically_stable(foot_info, body_pos) # Check if the robot is statically stable
            if stable: # Check if the robot is statically stable
                stability_sum += 1.0
            stability_samples += 1.0
            if print_stability:
                print(f"Feet touching ground: {foot_info}, we are {'stable' if stable else 'not stable'}")
        
        stability_multiplier = stability_sum / stability_samples # Percentage of the time the robot was stable applied as a multiplier
        THIS_EVAL = max_distance * stability_multiplier # Fitness function 
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
    ext = ""  # No extension your checkpoints don't have one
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
