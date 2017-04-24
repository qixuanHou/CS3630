#!/usr/bin/env python3

''' Get a raw frame from camera and display in OpenCV
By press space, save the image from 001.bmp to ...
'''

import cv2
import cozmo
from cozmo.util import Pose, distance_inches, degrees
import numpy as np
from find_ball import find_ball
from numpy.linalg import inv
import threading

from ar_markers.hamming.detect import detect_markers

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *


# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

#marker size in inches
marker_size = 3.5

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False

# goal location for the robot to drive to, (x, y, theta)
goal = (26, 9, 0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid)



async def image_processing(robot):

    global camK, marker_size

    event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # convert camera ima    ge to opencv format
    opencv_image = np.asarray(event.image)
    ball = find_ball(opencv_image)

    # detect markers
    markers = detect_markers(opencv_image, marker_size, camK)

    # show markers
    for marker in markers:
        marker.highlite_marker(opencv_image, draw_frame=True, camK=camK)
        #print("ID =", marker.id);
        #print(marker.contours);
    cv2.imshow("Markers", opencv_image)

    return (markers, ball)

#calculate marker pose
def cvt_2Dmarker_measurements(ar_markers):

    marker2d_list = [];

    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0,0,1], [0,-1,0], [1,0,0]])
        R_2_2p = np.matrix([[0,-1,0], [0,0,-1], [1,0,0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        #print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2,0], R_2p_1p[0,0])

        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        print('x =', x, 'y =', y,'theta =', yaw)

        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x,y,math.degrees(yaw)))

return marker2d_list


#compute robot odometry based on past and current pose
def compute_odometry(curr_pose, cvt_inch=True):
    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees

    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / 25.6, dy / 25.6

    return (dx, dy, diff_heading_deg(curr_h, last_h))

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)

class RobotStates(Enum):
    LOCALIZING = 1
    TRAVELING = 2
    SHOOT = 3
    RESET = 0

def our_go_to_pose(robot, curr_pose, dx, dy, dh):
    # goto = curr_pose.define_pose_relative_this(Pose(distance_inches(dx), distance_inches(dy), distance_inches(0), angle_z=degrees(dh)))
    # robot.go_to_pose(goto)
    goto_x = curr_pose.position.x + distance_inches(dx)
    goto_y = curr_pose.position.y + distance_inches(dy)
    goto_h = curr_pose.rotation.angle_z + degrees(dh)
    robot.go_to_pose(Pose(goto_x, goto_y, distance_inches(0), angle_z=goto_h))




async def run(robot: cozmo.robot.Robot):

    global flag_odom_init, last_pose
    global grid, gui

    # start streaming
    robot.camera.image_stream_enabled = True
    #start particle filter
    pf = ParticleFilter(grid)

    ###################
    reqConfidentFrames = 10
    reqUnconfidentFrames = 10
    ############YOUR CODE HERE#################
    condition = True
    estimated = [0,0,0,False]
    ballObj = Robot(0, 0, 0)
    STATE = RobotStates.LOCALIZING
    trueVal = 0
    falseVal = 0
    dist_to_ball = None
    robotLen = 1

    while (condition):
        curr_pose = robot.pose
        img = image_processing(robot)
        (markers, ball) = cvt_2Dmarker_measurements(img)
        dist_to_ball = calcDistance(ball)
        if(dist_to_ball is not None and estimated[3]):
            ballx = estimated[0] + math.cos(estimated[2]) * dist_to_ball
            bally = estimated[1] + math.sin(estimated[2]) * dist_to_ball
            ballObj.set_pos(ballx, bally)
            #thinkg about false positives
            #go to pose calculation
            #How to make a pose??
            #rotation plus position
            #once localized we can tell what directino we are looking in and what position we are in
            #add to cozmo's pose robot.Pose.pose.position in the direction of robot.Pose.pose.rotation

        odom = compute_odometry(curr_pose)
        estimated = pf.update(odom, markers)
        gui.show_particles(pf.particles)
        gui.show_robot(ballObj)
        gui.show_mean(estimated[0], estimated[1], estimated[2], estimated[3])
        gui.updated.set()

        if STATE == RobotStates.LOCALIZING:

            robot.drive_wheels(-5, -5)

            if estimated[3]:
                trueVal += 1
            else:
                trueVal = 0
            if trueVal > reqConfidentFrames && dist_to_ball is not None:
                robot.stop_all_motors()
                STATE = RobotStates.TRAVELING

        if STATE == RobotStates.TRAVELING:
            h = math.degrees(math.atan2(goal[1] - estimated[1], goal[0] - estimated[0]))
            dx = ballObj.x - estimated[0] - (math.cos(h) * robotLen)
            dy = ballObj.y - estimated[1] - (math.sin(h) * robotLen)
            dh = h - estimated[2]
            our_go_to_pose(robot, curr_pose, dx, dy, dh)

            if not estimated[3]:
                falseVal += 1
            else:
                falseVal = 0
            if trueVal > reqUnconfidentFrames:
                robot.stop_all_motors()
                STATE = RobotStates.LOCALIZING

            STATE = RobotStates.SHOOT

        if STATE == RobotStates.SHOOT:
            pass
            STATE == RobotStates.RESET

        if STATE == RobotStates.RESET:
            pass
            STATE = RobotStates.TRAVELING

        last_pose = curr_pose
    ###################


class CozmoThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)

def calcDistance(ball, ballSize=40.0, focalLength=220.0):
    if ball is not None:
        return (focalLength/ball[2])*ballSize
    return None
if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)
    gui.start()
