#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import sys
sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose
from pose_transform import get_global_pose_from_local

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
	robot.drive_wheels(50, 0, duration = 6)

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""
	# ####
	# TODO: Empirically determine the distance between the wheels of the robot using
	# robot.drive_wheels() function. Write a comment that explains how you determined
	# it and any computation you do as part of this function.
	# ####

	# The distance between left and right wheels has been calculated in the following way using 'get_distance_between_wheels_exp' function
	#1) get_distance_between_wheels_exp function rotates one wheel of the robot with the constant speed - 50 mmps
	#2) Before running the program the robot has been lifted by its head so that only its back wheels are touching the ground
	#3) The program has been started and the robot has started rotating around its back right wheel as only left wheel has been spinning with 50 mmps
	#4) By experimenting the duration of 6s has been found that allowed robot to make a full circle spin
	#5) The Distance that robot's spinning wheel has made is 6*50 = 300mm
	#6) 300mm is the length of the circle that robot has made. The radius is 300/(2*pi) = 47.75 mm
	#7) Since the robot has been spinning around its static wheel, the radius of the circle equals to the distance between robot's left and right wheel

	return 47.75

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


	target_local_pose = cozmo.util.pose_z_angle(x, y, 0, degrees(angle_z))
	robot_pose = robot.pose
	target_pose = get_global_pose_from_local(robot_pose, target_local_pose)
	target_pose_x = target_pose.position.x
	target_pose_y = target_pose.position.y

	b = get_distance_between_wheels()
	r = get_front_wheel_radius()
	iteration_duration = 0.5
	epsilon = 10
	distance = 100

	while distance > epsilon:

		robot_pose = robot.pose
		robot_pose_x = robot_pose.position.x
		robot_pose_y = robot_pose.position.y
		robot_angle = robot_pose.rotation.angle_z.radians

		distance = math.sqrt((target_pose_x - robot_pose_x)**2 + (target_pose_y - robot_pose_y)**2)
		direction_deviation_angle = math.atan((robot_pose_y - target_pose_y)/(robot_pose_x - target_pose_x))
		if robot_pose_x > target_pose_x:
			if robot_pose_y > target_pose_y:
				direction_deviation_angle -= math.pi
			else:
				direction_deviation_angle += math.pi

		angle_to_target = robot_angle - direction_deviation_angle

		direct_velocity = 40
		if distance < 50:
			direct_velocity = 20

		print("Target X: " + str(target_pose_x))
		print("Target Y: " + str(target_pose_y))
		print("Robot X: " + str(robot_pose_x))
		print("Robot Y: " + str(robot_pose_y))
		print("Distance: " + str(distance))
		print("Robot angle: " + str(robot_angle))
		print("Direction deviation angle: " + str(direction_deviation_angle))
		print("Angle to target: " + str(angle_to_target))

		# sl = (2*direct_velocity - angle_to_target*b)/(2*r)
		# sr = (2*direct_velocity + angle_to_target*b)/(2*r)
		sl = direct_velocity - angle_to_target*b
		sr = direct_velocity + angle_to_target*b
		print("Velocity:")
		print(sl)
		print(sr)
		print("==============")
		robot.drive_wheels(sr, sl)


	# # Lengths of arcs for each wheel to reach (x,y)
	# # These are arcs of the circle with the center at (0,y)
	# if y > 0:
	# 	l1 = math.pi*(y-b/2) / 2
	# 	l2 = math.pi*(y+b/2) / 2
	# else:
	# 	y = abs(y)
	# 	l1 = math.pi * (y + b/2) / 2
	# 	l2 = math.pi * (y - b/2) / 2
    #
	# print("Length left: " + str(l1))
	# print("Length right: " + str(l2))
    #
	# # Speeds for each wheel
	# s1 = l1 / duration
	# s2 = l2 / duration
    #
	# if s1 > s2 :
	# 	s1 += 10 # some constant that takes warm up time into account. It was found experimentally
	# else:
	# 	s2 += 10
    #
	# print("Speed left: " + str(s1))
	# print("Speed right: " + str(s2))
    #
	# robot.drive_wheels(s1, s2, duration = duration)
	# time.sleep(0.1)
    #
	# After the location is reached, read the pose and turn in place towards needed angle
	cur_pose = robot.pose
	cur_angle = cur_pose.rotation.angle_z.degrees
	angle_to_turn =  angle_z - cur_angle
	print("Reached destination. Turning by: "  + str(angle_to_turn))
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


def run(robot: cozmo.robot.Robot):

	print("***** Front wheel radius: " + str(get_front_wheel_radius()))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))

	## Example tests of the functions

	#front_wheel_exp(robot)
	#get_distance_between_wheels_exp(robot)
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
	my_go_to_pose2(robot, -200, -200, 180)
	#my_go_to_pose2(robot, 300, 300, 45)
	# my_go_to_pose3(robot, 100, 100, 45)


if __name__ == '__main__':

	cozmo.run_program(run)



