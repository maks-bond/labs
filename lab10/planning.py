
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
import asyncio

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

        if coord == goal:
            print("reached finish")
            path = []
            cur = item
            while cur is not None:
                path.insert(0, cur.coord)
                cur = cur.parent
                #print(cur)

            grid.setPath(path)
            return

        grid.addVisited(coord)
        neighbors = grid.getNeighbors(coord)
        for neighbor in neighbors:
            neighbor_coord = neighbor[0]
            neighbor_dist = neighbor[1]
            if neighbor_coord not in grid.getVisited():

                value = distance+1.1*heuristic(neighbor_coord, goal)
                q.put((value, Node(neighbor_coord, item, distance+neighbor_dist)))

    pass # Your code here


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
        
    return math.sqrt(math.pow(current[0]-goal[0],2) + math.pow(current[1]-goal[1],2)) # Your code here


grid_to_mm = 20

# We are assuming that robot's start position is at global (0, 0) pose
def get_cube_pose_in_grid(cube_pose, grid):

    grid_x = round(cube_pose.position.x/grid.scale)
    grid_y = round(cube_pose.position.y/grid.scale)
    return (grid_x, grid_y)

def updateObstacles(grid, cube2_grid, cube3_grid):
    grid.clearObstacles()
    if cube2_grid is not None:
        grid.addObstacle(cube2_grid)
    if cube3_grid is not None:
        grid.addObstacle(cube3_grid)

def getCenterPose(grid):
    grid_center_x = grid.width/2.0
    grid_center_y = grid.height / 2.0
    #return cozmo.util.pose_z_angle(grid_center_x*grid.scale, grid_center_y*grid.scale, 0, cozmo.util.radians(0))
    return (grid_center_x, grid_center_y)

def get_robot_grid_coord(robot):
    grid_x = round(robot.pose.position.x / grid.scale)
    grid_y = round(robot.pose.position.y / grid.scale)
    return (grid_x, grid_y)

def move_to_grid_coord(robot, coord, next_coord):
    global_x = coord[0]*grid.scale
    global_y = coord[1] * grid.scale
    angle = 0
    if next_coord is not None:
        angle = math.atan2(next_coord[1] - coord[1], next_coord[0] - coord[0])

    print("Angle " + str(angle))
    pose = cozmo.util.pose_z_angle(global_x, global_y, 0, cozmo.util.radians(angle))
    robot.go_to_pose(pose, relative_to_robot=False).wait_for_completed()

def grid_coord_is_different(coord1, coord2):
    if coord1 == None and coord2 == None:
        return False

    if coord1 == None or coord2 == None:
        return True

    distance = math.sqrt(math.pow(coord1[0]-coord2[0],2) + math.pow(coord1[1]-coord2[1],2))
    return distance >= 1.5

def reset_astar(grid):
    #grid.clearGoal()
    grid.clearVisited()
    #grid.clearObstacles()
    grid.clearPath()

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

    grid.setStart((0, 0))
    current_robot_grid_coord = (0, 0)
    while not stopevent.is_set():
        # TODO: Support removal and addition of a cube
        try:
            observed_cube = robot.world.wait_for_observed_light_cube(timeout=.1, include_existing=False)
        except asyncio.TimeoutError:
            pass


        if looking_for_target and (current_path is None):
            center_pose = getCenterPose(grid)
            grid.addGoal(center_pose)
            astar(grid, heuristic)
            current_path = grid.getPath()
            reset_astar(grid)
            current_path_index = 0

        if looking_for_target and (not reached_center):
            current_coord = current_path[current_path_index]
            next_coord = current_path[current_path_index+1] if current_path_index+1 < len(current_path) else None
            print("Going to " + str(current_coord))
            move_to_grid_coord(robot, current_coord, next_coord)
            current_path_index+=1
            if current_path_index >= len(current_path):
                reached_center = True

        if looking_for_target and reached_center:
            # Rotate around
            print("Reached center, rotating 45 degrees")
            robot.turn_in_place(cozmo.util.degrees(45), speed=cozmo.util.degrees(30)).wait_for_completed()


        if (not looking_for_target) and not done and (current_path is None):
            print("Running A*")
            astar(grid, heuristic)
            current_path = grid.getPath()
            reset_astar(grid)
            print("Got path to target: " + str(current_path))
            current_path_index = 0

        if (not looking_for_target) and not done and (current_path is not None):
            current_coord = current_path[current_path_index]
            next_coord = current_path[current_path_index + 1] if current_path_index + 1 < len(current_path) else None
            print("Going to " + str(current_coord))
            move_to_grid_coord(robot, current_coord, next_coord)
            current_path_index += 1
            if current_path_index >= len(current_path):
                done = True


        if observed_cube is not None:
            observed_cube_id = observed_cube.cube_id
            if observed_cube_id == 1:
                cube1 = observed_cube
                new_cube1_grid = get_cube_pose_in_grid(observed_cube.pose, grid)

                looking_for_target = False

                if grid_coord_is_different(new_cube1_grid, cube1_grid):
                    # TODO: rerun A* if the goal has changed
                    cube1_grid = new_cube1_grid
                    grid.clearGoals()
                    grid.addGoal(cube1_grid)
                    grid.setStart(get_robot_grid_coord(robot))
                    current_path = None
                    print("Cube 1, with pose in grid: %s" % str(cube1_grid))
            if observed_cube_id == 2:
                cube2 = observed_cube
                new_cube2_grid = get_cube_pose_in_grid(observed_cube.pose, grid)
                if grid_coord_is_different(new_cube2_grid, cube2_grid):
                    cube2_grid = new_cube2_grid
                    updateObstacles(grid, cube2_grid, cube3_grid)
                    print("Cube 2, with pose: %s" % str(cube2_grid))
            if observed_cube_id == 3:
                cube3 = observed_cube
                new_cube3_grid = get_cube_pose_in_grid(observed_cube.pose, grid)
                if grid_coord_is_different(new_cube3_grid, cube3_grid):
                    cube3_grid = new_cube3_grid
                    updateObstacles(grid, cube2_grid, cube3_grid)
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

