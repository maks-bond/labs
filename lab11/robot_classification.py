#!/usr/bin/env python3

import asyncio
import sys

import cv2
import numpy as np

from imgclassification import ImageClassifier
import math
import cozmo
import time
from enum import Enum

classifier = None

def kick(robot: cozmo.robot.Robot):
    robot.move_lift(5)
    time.sleep(.2)
    robot.move_lift(-5)

def get_anim_name(label):
    if label == "order":
        return "anim_poked_giggle"
    if label == "truck":
        return "anim_upgrade_reaction_lift_01"
    if label == "drone":
        return "anim_bored_event_02"

    return "anim_pounce_success_02"

async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    #add annotators for battery level and ball bounding box
    await robot.set_head_angle(cozmo.util.Angle(degrees=30)).wait_for_completed()

    classifier = ImageClassifier()
    (train_raw, train_labels) = classifier.load_data_from_folder('./test/')
    train_data = classifier.extract_image_features(train_raw)
    classifier.train_classifier(train_data, train_labels)

    counter_threshold = 5
    counter = counter_threshold
    current_label = 'none'
    try:

        while True:
            counter-=1
            if counter < 0:
                current_label = 'none'

            #get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
            img_features = classifier.extract_image_features([opencv_image])
            label = classifier.predict_labels(img_features)[0]
            if label is not 'none':
                counter = counter_threshold
                current_label = label

            if current_label != "none":
                await robot.say_text(current_label).wait_for_completed()
                anim_name = get_anim_name(label)
                await robot.play_anim(name=anim_name).wait_for_completed()
                await robot.set_head_angle(cozmo.util.Angle(degrees=30)).wait_for_completed()

            print(label)

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
