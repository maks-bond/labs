#!/usr/bin/env python3

import asyncio
import sys

import re
import cv2
import numpy as np

from imgclassification import ImageClassifier
from skimage import io
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
    if label == "inspection":
        return "anim_poked_giggle"
    if label == "truck":
        return "anim_upgrade_reaction_lift_01"
    if label == "drone":
        return "anim_bored_event_02"

    return "anim_pounce_success_02"


def imread_convert(f):
    return io.imread(f).astype(np.uint8)

def load_data_from_folder(dir, accepted_labels):
    # read all images into an image collection
    ic = io.ImageCollection(dir + "*.bmp", load_func=imread_convert)

    # create one large array of image data
    data = io.concatenate_images(ic)

    # extract labels from image names
    labels = np.array(ic.files)
    idx = 0
    indexes = []
    out_labels = []
    for i, f in enumerate(labels):
        m = re.search("_", f)
        label = f[len(dir):m.start()]
        if label in accepted_labels:
            out_labels.append(label)
            indexes.append(idx)
        idx+=1

    data = data[indexes]
    return (data, out_labels)

# When collect_images is True robot switches to collection mode where it saves every 10th frame to 'robotimages' folder. It uses 'collection_label' as a filename
collect_images = False
collection_label = 'inspection'

async def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    #add annotators for battery level and ball bounding box
    await robot.set_head_angle(cozmo.util.Angle(degrees=30)).wait_for_completed()

    # If we are not collecting images, let's train the classifier
    # The classifier is trained on images that have been collected separately with robot's camera in the same environment where testing takes place
    # Around 50 images of each class have been collected in '/robotimages' folder
    # The image collection in the environment that is similar to testing environment has made the recognotion much more robust
    if collect_images == False:
        classifier = ImageClassifier()
        (train_raw, train_labels) = classifier.load_data_from_folder('./robotimages/')
        train_data = classifier.extract_image_features(train_raw)
        classifier.train_classifier(train_data, train_labels)

    # counter is used to decide when robot should spell label and perform an animation
    counter = 0
    current_label = 'none'
    last_label = 'none'
    save_counter = 0
    try:
        while True:
            #get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            image = np.asarray(event.image)
            # If in image collection mode, save each 10th frame
            if collect_images == True:
                if save_counter % 10 == 0:
                    label = collection_label
                    file_name = "robotimages/"+ label + "_" + str(save_counter//10) + ".bmp"
                    io.imsave(file_name, image)
                    print("Saved image with file name " + file_name)

                save_counter +=1

            # Prediction phase, use trained classifer to predict image label
            # If the label has been predicted 20 times in a row, consider the prediction to be stable and spell label and perform appropriate animation
            if collect_images == False:
                img_features = classifier.extract_image_features([image])
                label = classifier.predict_labels(img_features)[0]

                # current_label equals to 'stable' recognized label
                if  current_label != "none":
                    await robot.say_text(current_label).wait_for_completed()
                    anim_name = get_anim_name(label)
                    await robot.play_anim(name=anim_name).wait_for_completed()
                    await robot.set_head_angle(cozmo.util.Angle(degrees=30)).wait_for_completed()
                    counter = 0
                    current_label = 'none'
                    last_label = label

                # If robot classifies same label increase the counter
                if last_label == label and (label != 'none'):
                    counter += 1
                    if counter >= 20:
                        current_label = label
                    print(counter)
                # Otherwise reset counter and label states
                else:
                    last_label = label
                    counter = 0
                    current_label = 'none'

                print(label)

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
