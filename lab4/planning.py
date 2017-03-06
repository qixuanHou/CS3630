
#author1: Ramamurthy Siripuram
#author2: Daniel Ocano

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo
from time import sleep
# import cozmo.world
# import cozmo.objects.BaseObject
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
    robot.image_stream_enabled = True
    global grid, stopevent

    robot.set_head_angle(radians(-0.23)).wait_for_completed()
    gridscale = 25
    scale = 27
    cube1id = 5
    grid.setStart((2, 2))
    grid.addGoal((13, 9))
    curSpot = grid.getStart() #hpefully valid
    curHeading = 0
    objects = set()
    cube1pos = None
    while not stopevent.is_set():
        # print([ e.pose.position for e in robot.world.visible_objects])
        # if world.light_cubes[cozmo.objects.LightCube1Id].is_visible
        for e in robot.world.visible_objects:
            if e.object_id not in objects:
                xr = curSpot[0]
                yr = curSpot[1]
                xc = e.pose.position.x
                yc = e.pose.position.y
                # yc = yr + ((e.pose.position.x * math.sin(curHeading)) + (e.pose.position.y * math.cos(curHeading)))/scale
                print(e.pose.position)
                xc = math.ceil((xc+50)/gridscale)
                yc = math.ceil((yc+50)/gridscale)
                if xc == 27:
                    xc = 26
                if yc == 27:
                    yc = 26
                print("xr: %d, yr: %d" % (xr, yr))
                print("xc: %d, yc: %d" % (xc, yc))
                print(curHeading/math.pi)

                if xc <= 26 and yc <= 18:
                    for i in [-2,-1, 0 , 1, 2]:
                        for j in [-2,-1, 0, 1,2]:
                            xci = xc + i
                            ycj = yc + j
                            if xci <= 26 and xci >= 0 and ycj <= 18 and ycj >= 0:
                                grid.addObstacle((xci,ycj))
                    print("Obstacle pos", xc,yc)
                    objects.add(e.object_id) #you ge tthe idea
                    if robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == e.object_id:
                        cube1pos = (xc,yc)
                        print("e id ",e.object_id)
                        grid.clearGoals()
                        angelz = e.pose.rotation.angle_z.radians
                        print("angelz+mod",angelz,angelz+math.pi)
                        angelz = math.pi + angelz
                        xc += 3*round(math.cos(angelz))
                        yc += 3*round(math.sin(angelz))
                        grid.addGoal((xc, yc))
                    # else:
                #     make it an obstacle
        # if not cubeFound:
        sleep(.4)
        astar(grid, heuristic)
        path = grid.getPath()
        print([e for e in grid.getGoals()])
        if(len(path) is 1):
            b = path[0]
            print("GOOOOOOOOAAAL")
        else:
            b = path[1]
        a = path[0]
        step = (b[0]-a[0], b[1]-a[1])
        step_len = math.sqrt(math.pow(a[0]-b[0],2) + math.pow(a[1]-b[1],2))
        heading = math.atan2(step[1], step[0])
        turnHeading = heading - curHeading
        curHeading = heading
        robot.turn_in_place(radians(turnHeading)).wait_for_completed()
        robot.drive_wheels(scale*step_len, scale*step_len, duration = 1.75*math.sqrt(step_len))
        grid.clearVisited()
        curSpot = b
        grid.setStart(b)
        if step_len == 0 and cube1pos is None:
            robot.turn_in_place(radians(math.pi/16)).wait_for_completed()
        elif step_len == 0 and cube1pos is not None:

            heading = math.atan2(cube1pos[1] - b[1], cube1pos[0] - b[0])
            print("b", b)
            print("cube1pos", cube1pos)
            print("heading", heading)
            turnHeading = heading - curHeading
            curHeading = heading
            robot.turn_in_place(radians(turnHeading)).wait_for_completed()
            break
        # # pass # Your code here


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def start(self):
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
