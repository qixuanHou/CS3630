
#author1: Ramamurthy Siripuram
#author2: Daniel Ocano

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo

from cozmo.util import radians, distance_mm, speed_mmps

def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """

    # discovered = list()
    pq = PriorityQueue()
    e = (1, [(grid.getStart(),0)])
    pq.put(e)
    while not pq.empty():
        path = pq.get()
        path = path[1]
        n = path[-1][0]
        if n in grid.getGoals():
            cpath = [p[0] for p in path]
            grid.setPath(cpath)
            return cpath
        if n not in grid.getVisited():
            grid.addVisited(n)
            for a in grid.getNeighbors(n):
                if a[0] not in grid.getVisited():
                    a = (a[0], a[1]+path[-1][1])
                    sucPath = path[:]
                    sucPath.append(a)
                    cpath = [p[0] for p in sucPath]
                    grid.setPath(cpath)
                    e = (heuristic(sucPath[-1], grid.getGoals()[0]), sucPath)
                    pq.put(e)
    return list()

def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """
    curpos = current[0]
    # out = current[1] * math.sqrt(math.pow(curpos[0]-goal[0][0],2) + math.pow(curpos[1]-goal[0][1],2))
    # out = current[1] + math.sqrt(abs(curpos[0]-goal[0][0]) + abs(curpos[1]-goal[0][1]))
    out = current[1] + math.sqrt(math.pow(curpos[0]-goal[0],2) + math.pow(curpos[1]-goal[1],2))

    # print(out)
    return out


def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment document for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """

    global grid, stopevent

    grid.setStart((1, 1))
    grid.addGoal((9, 13))
    curHeading = 0
    while not stopevent.is_set():
        cubeFound = False


        if not cubeFound:
            path = grid.getPath()
            a = path[0]
            b = path[1]
            step = (b[0]-a[0], b[1]-a[1])
            step_len = math.sqrt(math.pow(a[0]-b[0],2) + math.pow(a[1]-b[1],2))
            if step_len == 0:
                print("found it")
                break
            heading = math.atan2(step[1], step[0])
            curHeading = heading - curHeading
            robot.turn_in_place(radians(curHeading)).wait_for_completed()
            robot.drive_straight(distance_mm(25*step_len), speed_mmps(25*step_len)).wait_for_completed()
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

