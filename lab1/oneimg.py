import cv2
import sys
import copy
from find_ball import find_ball
from find_ball import display_circles

import numpy as np

def test_single_image(imageNumber, debug = False):
    #I am sure there is an easier way to do this, but this is quicker
    with open('./imgs/ground_truth.txt') as f:
        grid_data = [i.split() for i in f.readlines()]
    if len(grid_data) - 1 < imageNumber or imageNumber < 0:
        print("image number out of range!")
        return -1
    fileData = grid_data[imageNumber]
    file = fileData[0]
    opencv_image = cv2.imread("./imgs/" + file, cv2.COLOR_GRAY2RGB)
    #this calls your find_ball and allows you pass your own debug
    # a = 0.5
    # b = 50
    # img = (opencv_image*a)+b
    # np.clip(img, 0, 255, out=img)
    # img = img.astype('uint8')
    ball = find_ball(opencv_image, debug=debug)
    display_circles(opencv_image, [ball])
    if ball is not None:
        print("single image test returned: " + file, ball[0],ball[1],ball[2])
        print("expecting: " + fileData[0],fileData[1],fileData[2],fileData[3])
    else:
        print("the ball returned is none")

def find_image_by_number(imageNumber):
    with open('./imgs/ground_truth.txt') as f:
        grid_data = [i.split() for i in f.readlines()]
    if imageNumber % 2 == 0:
        print("warning: you asked for an even number, might be your problem since none are in ground_truth.txt")
    #very simple search to find image
    image_location = -1
    current_location = 0
    imageNumberString = str(imageNumber)
    #to add the zero in front
    if imageNumber < 10:
        imageNumberString = "0" + imageNumberString;
    imageNumberString = "test" + imageNumberString + ".bmp"
    for data in grid_data:
        if data[0] == imageNumberString:
            image_location = current_location
        current_location += 1
    if (image_location == -1):
        print("image not found!")
    return image_location

for i in sys.argv[1:]:
    test_single_image(find_image_by_number(int(i)))