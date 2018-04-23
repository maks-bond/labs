#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius():
	"""Returns the radius of the Cozmo robot's front wheel in millimeters."""
	# ####
	# TODO: Empirically determine the radius of the robot's front wheel using the
	# cozmo_drive_straight() function. You can write a separate script for doing 
	# experiments to determine the radius. This function should return the radius
	# in millimeters. Write a comment that explains how you determined it and any
	# computation you do as part of this function.
	# ####

	# Front wheel radius has been emperically calculated by
	# 1) Aligning one of the three 'holes' of the front wheel to some particular position
	# 2) Trying out different values of the distance parameter and invoking function 'cozmo_drive_straight(robot, 85, 20)'
	# 3) 85 happened to be the value of a distance that causes the marker 'hole' to perform one full rotation. As a result the wheel has performed one full rotation
	# 4) 85 is the length of the circle. Than the radius of the front wheel is 85/(2*Pi) = 13.5 mm
	# front_wheel_exp function was used

	return 13.5

def front_wheel_exp(robot):
	cozmo_drive_straight(robot, 85, 20)

def get_distance_between_wheels_exp(robot):
	robot.drive_wheels(30, -30, duration = 20)

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""
	# ####
	# TODO: Empirically determine the distance between the wheels of the robot using
	# robot.drive_wheels() function. Write a comment that explains how you determined
	# it and any computation you do as part of this function.
	# ####
	pass

def rotate_front_wheel(robot, angle_deg):
	"""Rotates the front wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""
	full_wheel_rotation_fraction = angle_deg / 360
	distance = full_wheel_rotation_fraction*2*math.pi*get_front_wheel_radius()

	cozmo_drive_straight(robot, distance, 50)

def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""

	warm_up_duration = 0.6
	duration = warm_up_duration + dist/speed
	robot.drive_wheels(speed, speed, duration = duration)



def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""

	is_negative = angle < 0
	angle = abs(angle)
	# full_around_distance was emperically calculated
	full_around_distance = 290
	rotation_amount = angle / 360

	distance = full_around_distance * rotation_amount
	warm_up_duration = 0.6
	duration = warm_up_duration + distance / speed

	sl = -speed
	sr = speed
	if is_negative:
		sl = -sl
		sr = -sr

	robot.drive_wheels(sl, sr, duration = duration)


def my_go_to_pose1(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# using the my_drive_straight and my_turn_in_place functions. This should
	# include a sequence of turning in place, moving straight, and then turning
	# again at the target to get to the desired rotation (Approach 1).
	# ####

	distance = math.sqrt(x**2 + y**2)
	angle = math.degrees(math.atan2(y, x))

	# 1. Turn in place
	my_turn_in_place(robot, angle, 40)
	time.sleep(0.2)
	# 2. Go
	my_drive_straight(robot, distance, 40)
	# 3. Turn again to match angle_z
	angle = angle_z - angle

	time.sleep(0.2)
	my_turn_in_place(robot, angle, 40)

#def get_duration_from_angle_and_speed(robot, angle, speed)

def my_go_to_pose2(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# using the robot.drive_wheels() function to jointly move and rotate the 
	# robot to reduce distance between current and desired pose (Approach 2).
	# ####

	b = 50
	duration = 7
	l1 = math.pi*(y-b/2) / 2
	l2 = math.pi*(y+b/2) / 2

	s1 = l1 / duration
	s2 = l2 / duration + 10

	print(s1)
	print(s2)
	print(robot.pose)
	robot.drive_wheels(s1, s2, duration = duration)
	print(robot.pose)
	time.sleep(0.1)

	cur_pose = robot.pose
	cur_angle = cur_pose.rotation.angle_z.degrees
	angle_to_turn =  angle_z - cur_angle
	print("Turning by angle: " + str(angle_to_turn))
	my_turn_in_place(robot, angle_to_turn, 40)

def my_go_to_pose3(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# as fast as possible. You can experiment with the built-in Cozmo function
	# (cozmo_go_to_pose() above) to understand its strategy and do the same.
	# ####
	pass

def run(robot: cozmo.robot.Robot):

	print("***** Front wheel radius: " + str(get_front_wheel_radius()))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))

	## Example tests of the functions

	#front_wheel_exp(robot)
	# get_distance_between_wheels_exp(robot)
	# cozmo_drive_straight(robot, 62, 50)
	# cozmo_turn_in_place(robot, 180, 30)
	#cozmo_go_to_pose(robot, 100, 100, 90)

	#robot.drive_wheels(20, -20, duration = 13.8)
	#robot.drive_wheels(30, 30, duration = 3)
    #
	#rotate_front_wheel(robot, 120)
	# my_drive_straight(robot, 85*3, 50)
	#my_turn_in_place(robot, 45, 40)
	#time.sleep(0.1)
	#my_turn_in_place(robot, 45, 40)
    #
	#my_go_to_pose1(robot, 100, 100, 180)
	#cozmo_go_to_pose(robot, 300, 300, 45)
	my_go_to_pose2(robot, 300, 300, 45)
	# my_go_to_pose3(robot, 100, 100, 45)


if __name__ == '__main__':

	cozmo.run_program(run)



