
#author1:
#author2:

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo

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
            print(path)
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
    
    while not stopevent.is_set():
        pass # Your code here


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

