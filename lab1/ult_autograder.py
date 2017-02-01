#!/usr/bin/env python3

import cv2
import os
import math
import find_ball
import numpy as np

# load ground truth

with open('./imgs/ground_truth.txt') as f:
    grid_data = [i.split() for i in f.readlines()]

print(grid_data)



# thresh hold to accept circle and give credit per circle

center_err_thresh = 20.0
radius_err_thresh = 10.0

SAFE_center_err_thresh = center_err_thresh * 0.5
SAFE_radius_err_thresh = radius_err_thresh * 0.5

center_errs = []
radius_errs = []


unsafe_detections = []
incorrect_detections = []

autograder_score = 0;
safe_score = 0;

# check each image

for filedata in grid_data:

    file = filedata[0]



    #read in image as grayscale

    opencv_image = cv2.imread("./imgs/" + file, cv2.COLOR_GRAY2RGB)



    #try to find the ball in the image

    ball = find_ball.find_ball(opencv_image)



    no_ball = ball is None

    if no_ball:

        ball = np.array([0, 0, 0])



    # get center err

    center_err = math.sqrt((ball[0] - float(filedata[1]))**2 + (

        ball[1] - float(filedata[2]))**2)



    # get radius err

    r_err = math.fabs(ball[2] - float(filedata[3]))



    center_errs.append(center_err)

    radius_errs.append(r_err)



    print(file)
    tuple_ball = ball

    if no_ball:
        tuple_ball = "No ball detected"

    print("\t{0}".format(tuple_ball))

    print("\tcircle center err =", center_err, "pixel")

    print("\tcircle radius err =", r_err, "pixel")

    if center_err <= center_err_thresh and r_err <= radius_err_thresh:

        if center_err <= SAFE_center_err_thresh and r_err <= SAFE_radius_err_thresh:

            safe_score += 1

        else:

            unsafe_detections.append((file, tuple_ball, center_err, r_err))

        autograder_score += 1

    else:

        incorrect_detections.append((file, tuple_ball, center_err, r_err))

        circle = [ball]

        find_ball.display_circles(opencv_image, circle)

print("\nAverage center error was:", np.mean(center_errs))

print("Average radius error was:", np.mean(radius_errs))

print("\nHighest center error was:", np.amax(center_errs))

print("Highest radius error was:", np.amax(radius_errs))

print("\nUnsafe detections were as follows:")
if not len(unsafe_detections):
    print("\tNo unsafe detections.")

for i in range(len(unsafe_detections)):

    print("\t{0}: \n\t\t{1} \n\t\tcenter_err = {2} pixel \n\t\tradius_err = {3} pixel".format(unsafe_detections[i][0], unsafe_detections[i][1], unsafe_detections[i][2], unsafe_detections[i][3]))


print("Incorrect detections were as follows:", incorrect_detections)
if not len(incorrect_detections):
    print("\tNo incorrect detections.")
for i in range(len(incorrect_detections)):

    print("\t{0}: \n\t\t{1} \n\t\tcenter_err = {2} pixel \n\t\tradius_err = {3} pixel".format(incorrect_detections[i][0], incorrect_detections[i][1], incorrect_detections[i][2], incorrect_detections[i][3]))

print("\nAutograder score =", autograder_score, "\n\t(Autograder thresholds are",center_err_thresh, "for center and", radius_err_thresh, "for radius)")

print("Safe score =", safe_score, "\n\t(Safe thresholds were",SAFE_center_err_thresh, "for center and", SAFE_radius_err_thresh, "for radius)")