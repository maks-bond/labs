#!/usr/bin/env python3

import asyncio
import sys

import cv2
import numpy as np

sys.path.insert(0, '../lab4')
import find_ball
import math
import cozmo
import time
from enum import Enum

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')

class State(Enum):
    SEARCHING = 1,
    FOLLOWING = 2,
    KICKING = 3

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
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BallAnnotator.ball = None


def kick(robot: cozmo.robot.Robot):
    robot.move_lift(5)
    time.sleep(.2)
    robot.move_lift(-5)


async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    #add annotators for battery level and ball bounding box
    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)
    action = robot.set_head_angle(cozmo.util.Angle(degrees=0))
    action.wait_for_completed()

    lower_yellow = np.array([25,160,50])
    upper_yellow = np.array([30,255,255])
    not_observed_count = 0
    state = State.SEARCHING
    last_observed_ball = None

    try:

        while True:
            #get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            opencv_image = cv2.bitwise_not(opencv_image)
            #find the ball
            ball = find_ball.find_ball(opencv_image)

            if ball is not None:
                #if last_observed_ball is None:
                    #print("last_observed_ball was None. Found a ball.")
                last_observed_ball = ball

            #STATE TRANSITION
            if ball is None:
                not_observed_count+=1
            else:
                not_observed_count = 0
            if not_observed_count >= 5:
                #print("last_observed_ball becomes None")
                last_observed_ball = None

            if last_observed_ball is not None:
                #print("ball radius: %d" % ball[2])
                if last_observed_ball[2] > 80:
                    if state is not State.KICKING:
                        print("State is KICKING")
                    state = State.KICKING
                else:
                    if state is not State.FOLLOWING:
                        print("State is FOLLOWING")
                    state = State.FOLLOWING
            else:
                if state is not State.SEARCHING:
                    print("State is SEARCHING")
                state = State.SEARCHING

            #Standard logic to get speed when ball is visible
            w = opencv_image.shape[1]
            middle = w / 2
            if last_observed_ball is not None:
                ball_center_x = last_observed_ball[0]
                diff = ball_center_x - middle

                if abs(diff) < 10:
                    motor_right = 25
                    motor_left = 25
                elif diff > 0:
                    motor_right = 0
                    motor_left = 15
                else:
                    motor_right = 15
                    motor_left = 0
            #EXECUTION
            if state == State.SEARCHING:
                motor_right = 0
                motor_left = 15
            elif state == State.KICKING:
                kick(robot)
                motor_right = 25
                motor_left = 25


            #print("motor_right: %d" % motor_right)
            #print("motor_left: %d" % motor_left)
            await robot.drive_wheels(motor_left, motor_right)

            time.sleep(.2)

            #set annotator ball
            BallAnnotator.ball = ball

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)



if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)

