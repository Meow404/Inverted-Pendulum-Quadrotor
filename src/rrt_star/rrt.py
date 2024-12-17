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
    Check if the quadrotor collides with any obstacles, considering inflated boxes.
    :param start: Starting point coordinates of the quadrotor.
    :param end: Goal point coordinates of the quadrotor.
    :param map_struct: Map struct containing obstacles.
    :param padding: Padding distance to inflate obstacles.
    :return: True if collision detected, otherwise False.
    """
    
    initial_pos = start
    random_pos = end


    for box in map_struct.obstacles:
        # Inflate the box with the given padding
        inflated_box = inflate_box(box, padding)
        
        # Check for collision between the single 3D points and the inflated box
        if detectCollisionOnce(initial_pos, random_pos, inflated_box): 
            return True  # Collision detected

    return False

def nearest_node(nodes, sample):

    # Find nearest node index to the sample configuration
    distances = [np.linalg.norm(node["config"] - sample) for node in nodes]
    return np.argmin(distances)

def steer(from_node, to_sample, step_size=0.3):
    
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
        start = end
    
    return False


def shortcutting(path, map_struct):
    
    shortcut = [path[0]]
    i = 1

    while i < len(path)-1:
        shortcut.append(path[i])
        i += 1
        for j, config in enumerate(path[i:]):
            if not is_path_colliding(shortcut[-2], config, map_struct):
                shortcut[-1] = config
            else:

                i = i + j  
                break
        # if reached the end of path
        else:
            break
    return shortcut

def rrt(map_struct, start, goal):
    # Initialize tree and path
    tree = [{"config": start, "parent": None}]
    path = []

    if not is_path_colliding(start, goal, map_struct):
        return [start, goal]

    
    last_config = start

    # RRT Loop
    for i in range(20000):  # max iterations
        
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

    # Trace path from goal to start
    current_index = len(tree) - 1
    while current_index is not None:
        path.append(tree[current_index]["config"])
        current_index = tree[current_index]["parent"]
    
    # Reverse path to start->goal order
    path.reverse()

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