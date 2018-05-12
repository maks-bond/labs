
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import asyncio
import time

class Node:
    def __init__(self, coord, parent, distance):
        self.coord = coord
        self.parent = parent
        self.distance = distance

    def __lt__(self, other):
        return self.coord < other.coord

    def __eq__(self, other):
        return self.coord == other.coord

    def __str__(self):
        return str((self.coord, self.distance, self.parent))

def print_queue(q):
    for i in range(len(q.queue)):
        print(q.queue[i])

    print("=========")

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """

    q = PriorityQueue()
    start = grid.getStart()
    q.put((0, Node(start, None, 0)))
    goal = grid.getGoals()[0]
    print("A* Start is" + str(start) + ", goal is: " + str(goal))
    while not q.empty():
        item = q.get()[1]
        coord = item.coord
        distance = item.distance
        # Check if goal is reached
        if coord == goal:
            path = []
            cur = item
            while cur is not None:
                path.insert(0, cur.coord)
                cur = cur.parent
                #print(cur)

            grid.setPath(path)
            return

        # Mark current cell as visited and expand
        grid.addVisited(coord)
        neighbors = grid.getNeighbors(coord)
        for neighbor in neighbors:
            neighbor_coord = neighbor[0]
            neighbor_dist = neighbor[1]
            if neighbor_coord not in grid.getVisited():
                # A* function: current distance + 1.1*heuristic value
                value = distance+1.1*heuristic(neighbor_coord, goal)
                q.put((value, Node(neighbor_coord, item, distance+neighbor_dist)))

    pass # Your code here


# Just eucledian distance to the target
def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
        
    return math.sqrt(math.pow(current[0]-goal[0],2) + math.pow(current[1]-goal[1],2)) # Your code here


def get_cube_pose_in_grid(cube_pose, grid):

    grid_x = round(cube_pose.position.x/grid.scale)
    grid_y = round(cube_pose.position.y/grid.scale)
    return (grid_x, grid_y)

def getCenterPose(grid):
    grid_center_x = grid.width/2.0
    grid_center_y = grid.height / 2.0
    #return cozmo.util.pose_z_angle(grid_center_x*grid.scale, grid_center_y*grid.scale, 0, cozmo.util.radians(0))
    return (grid_center_x, grid_center_y)

def get_robot_grid_coord(robot):

    grid_x = round(robot.pose.position.x / grid.scale)
    grid_y = round(robot.pose.position.y / grid.scale)
    print("Getting robot grid coordinates: " + str(robot.pose) + " grid coord: " + str((grid_x, grid_y)))
    return (grid_x, grid_y)

# Perform one step in A* path
def move_to_grid_coord(robot, coord, next_coord, rotation_angle = None):
    global_x = coord[0]*grid.scale
    global_y = coord[1] * grid.scale
    angle = 0
    if next_coord is not None:
        angle = math.atan2(next_coord[1] - coord[1], next_coord[0] - coord[0])

    if rotation_angle is not None:
        angle = rotation_angle

    print("Angle " + str(angle))
    pose = cozmo.util.pose_z_angle(global_x, global_y, 0, cozmo.util.radians(angle))
    robot.go_to_pose(pose, relative_to_robot=False).wait_for_completed()

# Check if grid coordinates are significantly different
def grid_coord_is_different(coord1, coord2):
    if coord1 == None and coord2 == None:
        return False

    if coord1 == None or coord2 == None:
        return True

    distance = math.sqrt(math.pow(coord1[0]-coord2[0],2) + math.pow(coord1[1]-coord2[1],2))
    return distance >= 1.5

# Reset A* state
def reset_astar(grid, goal, start, goal_cube, obstacle_cube1, obstacle_cube2):
    grid.clearGoals()
    grid.clearVisited()
    grid.clearObstacles()
    grid.clearPath()
    grid.clearStart()

    grid.addGoal(goal)
    grid.setStart(start)

    if goal_cube != None:
        add_obstacles_around_cube(grid, goal_cube)

    if obstacle_cube1 != None:
        add_obstacles_around_cube(grid, obstacle_cube1)

    if obstacle_cube2 != None:
        add_obstacles_around_cube(grid, obstacle_cube2)

# Calculate target goal taking into account needed 10 cm distance from the cube and cube's orientation
def get_target_goal(cube_pose):
    x = cube_pose.position.x
    y = cube_pose.position.y
    angle = cube_pose.rotation.angle_z.radians
    target_x = x+100*math.cos(angle)
    target_y = y+100*math.sin(angle)
    grid_target_x = round(target_x / grid.scale)
    grid_target_y = round(target_y / grid.scale)
    return (grid_target_x, grid_target_y)

def get_final_angle(cube_pose):
    angle = cube_pose.rotation.angle_z.radians
    return angle - math.pi

def add_obstacles_around_cube(grid, cube_grid_coord):
    obstacles = []
    obstacles.append((cube_grid_coord[0]-1, cube_grid_coord[1]))
    obstacles.append((cube_grid_coord[0]+1, cube_grid_coord[1]))
    obstacles.append((cube_grid_coord[0], cube_grid_coord[1] - 1))
    obstacles.append((cube_grid_coord[0], cube_grid_coord[1] + 1))
    obstacles.append((cube_grid_coord[0] - 2, cube_grid_coord[1]))
    obstacles.append((cube_grid_coord[0] + 2, cube_grid_coord[1]))
    obstacles.append((cube_grid_coord[0], cube_grid_coord[1] - 2))
    obstacles.append((cube_grid_coord[0], cube_grid_coord[1] + 2))

    obstacles.append((cube_grid_coord[0] - 1, cube_grid_coord[1] - 1))
    obstacles.append((cube_grid_coord[0] + 1, cube_grid_coord[1] - 1))
    obstacles.append((cube_grid_coord[0] - 1, cube_grid_coord[1] + 1))
    obstacles.append((cube_grid_coord[0] + 1, cube_grid_coord[1] + 1))

    obstacles.append((cube_grid_coord[0] - 2, cube_grid_coord[1] - 1))
    obstacles.append((cube_grid_coord[0] + 2, cube_grid_coord[1] - 1))
    obstacles.append((cube_grid_coord[0] - 2, cube_grid_coord[1] + 1))
    obstacles.append((cube_grid_coord[0] + 2, cube_grid_coord[1] + 1))

    obstacles.append((cube_grid_coord[0] - 1, cube_grid_coord[1] - 2))
    obstacles.append((cube_grid_coord[0] + 1, cube_grid_coord[1] - 2))
    obstacles.append((cube_grid_coord[0] - 1, cube_grid_coord[1] + 2))
    obstacles.append((cube_grid_coord[0] + 1, cube_grid_coord[1] + 2))

    obstacles.append((cube_grid_coord[0] - 2, cube_grid_coord[1] - 2))
    obstacles.append((cube_grid_coord[0] + 2, cube_grid_coord[1] - 2))
    obstacles.append((cube_grid_coord[0] - 2, cube_grid_coord[1] + 2))
    obstacles.append((cube_grid_coord[0] + 2, cube_grid_coord[1] + 2))

    grid.addObstacles(obstacles)

def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """
        
    global grid, stopevent

    robot.move_lift(-3)
    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    cube1 = None
    cube2 = None
    cube3 = None
    cube1_grid = None
    cube2_grid = None
    cube3_grid = None
    observed_cube = None
    looking_for_target = True
    current_path_index = None
    current_path = None
    reached_center = False
    done = False
    final_angle = None
    need_to_reset_astar = False

    grid.setStart((0, 0))
    while not stopevent.is_set():
        # TODO: Support removal and addition of a cube
        try:
            observed_cube = robot.world.wait_for_observed_light_cube(timeout=.1, include_existing=False)
        except asyncio.TimeoutError:
            pass

        # The logic to reach the center of the grid and rotate
        # Run A* and build the path and reset the state
        if looking_for_target and (current_path is None or need_to_reset_astar):
            print("Running A* to reach center of a grid")
            center_pose = getCenterPose(grid)
            reset_astar(grid, center_pose, get_robot_grid_coord(robot), cube1_grid, cube2_grid, cube3_grid)
            astar(grid, heuristic)
            current_path = grid.getPath()
            current_path_index = 0
            need_to_reset_astar = False

        # Perform next step following A* path
        if looking_for_target and (not reached_center):
            current_coord = current_path[current_path_index]
            next_coord = current_path[current_path_index+1] if current_path_index+1 < len(current_path) else None
            print("Going to " + str(current_coord))
            move_to_grid_coord(robot, current_coord, next_coord)
            current_path_index+=1
            if current_path_index >= len(current_path):
                reached_center = True

        # Once the center is reached rotate to be able to discover target cube
        if looking_for_target and reached_center:
            # Rotate around
            print("Reached center, rotating 30 degrees")
            robot.turn_in_place(cozmo.util.degrees(30), speed=cozmo.util.degrees(30)).wait_for_completed()
            time.sleep(1)

        # The logic to reach target position
        # Running A* and resetting the state
        if (not looking_for_target) and not done and (current_path is None or need_to_reset_astar):
            print("Running A* to reach target")
            reset_astar(grid, goal_grid, get_robot_grid_coord(robot), cube1_grid, cube2_grid, cube3_grid)
            astar(grid, heuristic)
            current_path = grid.getPath()
            print("Got path to target: " + str(current_path))
            current_path_index = 0
            need_to_reset_astar = False

        # Perform next step following A* path
        if (not looking_for_target) and not done and (current_path is not None):
            current_coord = current_path[current_path_index]
            next_coord = current_path[current_path_index + 1] if current_path_index + 1 < len(current_path) else None
            print("Going to " + str(current_coord))
            move_to_grid_coord(robot, current_coord, next_coord, final_angle if (current_path_index + 1)>= len(current_path) else None)
            current_path_index += 1
            if current_path_index >= len(current_path):
                done = True

        # Finished. Running animation to celebrate
        if done:
            robot.play_anim(name="anim_poked_giggle").wait_for_completed()


        # Cube observation logic
        # Every time new cube is observed or its position has significantly changed, we update cube's grid coordinates and set the flag to rerun A*
        if observed_cube is not None:
            observed_cube_id = observed_cube.cube_id
            if observed_cube_id == 1:
                cube1 = observed_cube
                new_cube1_grid = get_cube_pose_in_grid(observed_cube.pose, grid)

                looking_for_target = False

                if grid_coord_is_different(new_cube1_grid, cube1_grid):
                    cube1_grid = new_cube1_grid
                    goal_grid = get_target_goal(cube1.pose)
                    final_angle = get_final_angle(cube1.pose)
                    current_path = None
                    need_to_reset_astar = True
                    add_obstacles_around_cube(grid, cube1_grid)
                    print("Cube 1, with pose in grid: %s" % str(cube1_grid))
                    print("Target with pose in grid: %s" % str(goal_grid))
            if observed_cube_id == 2:
                # TODO: Rerun A* if an obstacle has been added
                cube2 = observed_cube
                new_cube2_grid = get_cube_pose_in_grid(observed_cube.pose, grid)
                if grid_coord_is_different(new_cube2_grid, cube2_grid):
                    cube2_grid = new_cube2_grid
                    need_to_reset_astar = True
                    add_obstacles_around_cube(grid, cube2_grid)
                    print("Cube 2, with pose: %s" % str(cube2_grid))
            if observed_cube_id == 3:
                cube3 = observed_cube
                new_cube3_grid = get_cube_pose_in_grid(observed_cube.pose, grid)
                if grid_coord_is_different(new_cube3_grid, cube3_grid):
                    cube3_grid = new_cube3_grid
                    need_to_reset_astar = True
                    add_obstacles_around_cube(grid, cube3_grid)
                    print("Cube 3, with pose: %s" % str(cube3_grid))


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

