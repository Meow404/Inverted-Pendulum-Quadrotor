import numpy as np
import random
from detectCollision import detectCollisionOnce
from loadmap import loadmap
from copy import deepcopy
#from lib.calculateFK import FK 
from tqdm import tqdm 
from time import perf_counter 
    

def inflate_box(box, padding):
    """
    Inflate a box by a given padding on all sides.
    :param box: Original box [xmin, ymin, zmin, xmax, ymax, zmax].
    :param padding: Distance to inflate the box.
    :return: Inflated box [xmin, ymin, zmin, xmax, ymax, zmax].
    """
    return [
        box[0] - padding,  # xmin
        box[1] - padding,  # ymin
        box[2] - padding,  # zmin
        box[3] + padding,  # xmax
        box[4] + padding,  # ymax
        box[5] + padding   # zmax
    ]

def isRobotCollided(start, end, map_struct, padding=1.3):
    """
    Check if a given robot configuration collides with any obstacles, considering inflated boxes.
    :param q: Joint configuration of the robot (1x7 array).
    :param map_struct: Map struct containing obstacles.
    :param padding: Padding distance to inflate obstacles.
    :return: True if collision detected, otherwise False.
    """
    
    '''
    # Convert q into a 2D array format for detectCollision compatibility
    q_joint_positions, _ = fk.forward(q)
    parent_joint_positions,  _ = fk.forward(parent)
    '''

    initial_pos = start
    random_pos = end


    for box in map_struct.obstacles:
        # Inflate the box with the given padding
        inflated_box = inflate_box(box, padding)
        
        # Check for collision between the single 3D point `q_points` and the inflated box
        if detectCollisionOnce(initial_pos, random_pos, inflated_box):  # Using q_points as both start and end to form a "line"
            #print("detected collision")
            return True  # Collision detected

    return False


'''
def sample_joint_config(q, goal, lowerLim, upperLim):

    #TODO: check if you need to modify this

    mean = q + (goal - q)/np.linalg.norm(goal - q)
    cfg = np.random.normal(mean, np.ones(7))
    cfg = np.clip(cfg, lowerLim, upperLim)
    #return np.array([random.uniform(lower, upper) for lower, upper in zip(lowerLim, upperLim)])
    return cfg 
'''

def nearest_node(nodes, sample):

    #This function does not have to be changed

    # Find nearest node index to the sample configuration
    distances = [np.linalg.norm(node["config"] - sample) for node in nodes]
    return np.argmin(distances)

def steer(from_node, to_sample, step_size=0.3):
    
    #This function does not have to be changed

    # Move a small step from from_node towards to_sample
    direction = to_sample - from_node
    length = np.linalg.norm(direction)
    return from_node + (direction / length) * min(step_size, length)

def is_path_colliding(from_node, to_node, map_struct):
    start = from_node
    end = from_node
    while not np.linalg.norm(to_node - start) < 1e-2:
        end = steer(start, to_node, 0.1)
        if isRobotCollided(start, end, map_struct):
            return True#, []
        #path.append([end])
        start = end
    
    #print("Shortcut not colliding") 
    return False


def shortcutting(path, map_struct):
    
    #I believe this function should remain unchanged

    shortcut = [path[0]]
    i = 1 # track norma path
    #print(len(path))

    while i < len(path)-1:
        #print(i)
        shortcut.append(path[i])
        i += 1
        for j, config in enumerate(path[i:]):
            if not is_path_colliding(shortcut[-2], config, map_struct):
                shortcut[-1] = config
                #print(shortcut)
            else:

                i = i + j  
                #print(i)
                break
        # if reached the end of path
        else:
            break
    #print(shortcut)
    return shortcut

'''
def is_joint_limits_violated(q, lower_limit, upper_limit):
    if ((q < lower_limit) | (q > upper_limit)).any():
        return True
    return False
'''

'''
def is_robot_in_obstacle(fk, q, obstacles):
    q_joint_positions, _ = fk.forward(q)
    for obstacle in obstacles:
        if any(detectCollision(q_joint_positions, q_joint_positions, obstacle)):
            return True
    return False
'''

def rrt(map_struct, start, goal):
    # Initialize tree and path
    tree = [{"config": start, "parent": None}]
    path = []

    '''
    # Joint limits
    lowerLim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upperLim = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    fk = FK()

    if is_joint_limits_violated(start, lowerLim, upperLim) \
        or is_joint_limits_violated(goal, lowerLim, upperLim) \
        or is_robot_in_obstacle(fk, start, map_struct.obstacles) \
        or is_robot_in_obstacle(fk, goal, map_struct.obstacles):
        return []
    '''


    if not is_path_colliding(start, goal, map_struct):
        return [start, goal]

    
    last_config = start

    # RRT Loop
    for i in range(20000):  # max iterations
        
        '''
        #print(i)
        # Randomly sample configuration within limits
        sample = sample_joint_config(last_config ,goal, lowerLim, upperLim) if random.random() > 0.1 else goal
        
        '''

        #Getting a random point with a 10% probability of bias towards goal direction
        if random.random()>0.1:

            random_x = random.uniform(-10, 10)
            random_y = random.uniform(-10, 10)
            random_z = random.uniform(-10, 10)
            sample = np.array([random_x, random_y, random_z])
        
        else:

            sample = goal
        
        # Find nearest node in tree to sample
        nearest_index = nearest_node(tree, sample)
        nearest_node_config = tree[nearest_index]["config"]
        
        # Steer towards sample
        new_config = steer(nearest_node_config, sample, 0.1)
        
        # Check if new config is in collision
        if not isRobotCollided(new_config, nearest_node_config, map_struct):
            tree.append({"config": new_config, "parent": nearest_index})
            last_config = new_config
            
            # Check if goal is reached
            if np.linalg.norm(new_config - goal) < 0.1:
                tree.append({"config": goal, "parent": len(tree) - 1})
                break
        #print(new_config)

    # Trace path from goal to start
    current_index = len(tree) - 1
    while current_index is not None:
        path.append(tree[current_index]["config"])
        current_index = tree[current_index]["parent"]
    
    # Reverse path to start->goal order
    path.reverse()

    #print(f"converged : {path}, {len(path)}")

    path = shortcutting(path, map_struct)
    
    return np.array(path) if path[0] is start and path[-1] is goal else np.array([])

if __name__ == '__main__':
    
    map_struct = loadmap("map1.txt")
    start = np.array([-7, -4, 0])
    goal = np.array([7, 3, 0])

    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))

    if len(path) > 0:
        print("Path exists")
        print(path)

    else:
        print("Path does not exist")
    
    '''
    # potential field planning
    timings = []
    success = 0
    for _ in tqdm(range(100)):
        dstart = perf_counter()
        q_path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
        status = len(q_path)!= 0 and (all(q_path[0] == start) and all(q_path[-1] == goal))
        dstop = perf_counter()
        dt = dstop - dstart
        if status:
            timings.append(dt)
            success += 1
    print(np.mean(timings))
    print(np.median(timings))
    print(np.std(timings))
    print(success)
    '''
