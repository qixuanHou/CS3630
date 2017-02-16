#!/usr/bin/env python3
#Ramamurthy Siripuram and Daniel Ocano
import math
import asyncio
import sys

import cv2
import numpy as np
import find_ball

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps
try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    sys.exit("Cannot import from PIL. Do `pip3 install --user Pillow` to install")
from state import State

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')



def calcDistance(ball, ballSize=40.0, focalLength=220.0):
    if ball is not None:
        return (focalLength/ball[2])*ballSize
    return None

# Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)

# Define a decorator as a subclass of Annotator; displays the ball
class BallAnnotator(cozmo.annotate.Annotator):

    ball = None
    distance = None
    # direction = None
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BallAnnotator.ball is not None:

            #double size of bounding box to match size of rendered image
            BallAnnotator.ball = np.multiply(BallAnnotator.ball,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BallAnnotator.ball[0]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[1]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[2]*2, BallAnnotator.ball[2]*2)

            # text = "find_ball: "+ ("%.2f" % calcDistance(BallAnnotator.ball))
            text = "find_ball: "+ ("%.2f" % BallAnnotator.distance)# + (" %.2f" % BallAnnotator.direction)
            imtx = cozmo.annotate.ImageText(text)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=imtx)

            BallAnnotator.ball = None

def image(text_to_draw, x=8, y=6, font=None):
    '''Make a PIL.Image with the given text printed on it

    Args:
        text_to_draw (string): the text to draw to the image
        x (int): x pixel location
        y (int): y pixel location
        font (PIL.ImageFont): the font to use

    Returns:
        :class:(`PIL.Image.Image`): a PIL image with the text drawn on it
    '''

    # make a blank image for the text, initialized to opaque black
    text_image = Image.new('RGBA', cozmo.oled_face.dimensions(), (0, 0, 0, 255))

    # get a drawing context
    dc = ImageDraw.Draw(text_image)

    # draw the text
    dc.text((x, y), text_to_draw, fill=(255, 255, 255, 255), font=font)

    out = cozmo.oled_face.convert_image_to_screen_data(text_image)
    return out
async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    #add annotators for battery level and ball bounding box
    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)

    try:
        state = State()
        trigger = True
        isBallFound = False
        isRobotAtBall = False

        # prevPos = None
        # direction = None
        while trigger:
            #get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            h, w = opencv_image.shape
            # print(w)
            #find the ball
            ball = find_ball.find_ball(opencv_image)
            distance = calcDistance(ball)
            # if prevPos

            #set annotator ball
            BallAnnotator.ball = ball
            BallAnnotator.distance = distance
            # BallAnnotator.direction = direction
            robot.display_oled_face_image(image(state.cur), 1000.0, in_parallel = True)
            if state.isCurState("START"):
                #spin around and search for ball
                #Make a sound and print something on screen

                # display for 1 second

                if ball is None:
                    await robot.drive_wheels(17,-17)
                else:
                    await robot.drive_wheels(0, 0, 0.5)
                    state.next()

            if state.isCurState("TRAVELING"):
                #Print and sound off
                # move towards ball
                if distance is None:
                    if left:
                        lspeed = 5
                        rspeed = 2*base
                    if right:
                        lspeed = 2*base
                        rspeed = 5
                    await robot.drive_wheels(lspeed,rspeed)
                if distance is not None:
                    if distance > 85:
                        base = 25
                        adj = (ball[0]-(w/2)) / (distance**0.5)
                        # print(distance)
                        # print("adj:", adj)
                        left = adj < -0.75
                        right = adj > 0.75

                        if left:
                            lspeed = base
                            rspeed = base - adj
                            print("LEFT")
                        elif right:
                            lspeed = base + adj
                            rspeed = base
                            print("RIGHT")
                        else:
                            lspeed = base + 20
                            rspeed = base + 20


                        await robot.drive_wheels(lspeed, rspeed)
                    else:
                        state.next()

            if state.isCurState("END"):
                #tap ball
                #Screen and sound off
                # await robot.drive_wheels(10,10, duration=1)
                # print("done yay")
                await robot.set_lift_height(1)
                await robot.set_lift_height(0)
                if distance is not None:
                    if left or right:
                        state.next()
                    if distance > 85:
                        state.next()

            if state.isCurState("PAUSE"):
                #pause for a moment
                await robot.drive_wheels(0, 0, 0.05)
                state.next()

                """if distance is not None:
                    isBallFound = True
                    await robot.drive_wheels(0,0, duration=0.1)"""
            # print("speed:", robot.left_wheel_speed.speed_mmps, robot.right_wheel_speed.speed_mmps)
            # print("ball x pos:", ball[0] if ball is not None else "None")

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
