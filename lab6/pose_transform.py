#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy
from cozmo.util import degrees
from cozmo.util import radians
import math

def get_relative_pose(object_pose, reference_frame_pose):
	# ####
	# TODO: Implement computation of the relative frame using numpy.
	# Try to derive the equations yourself and verify by looking at
	# the books or slides bfore implementing.
	# ####

	obj_x = object_pose.position.x
	obj_y = object_pose.position.y
	obj_z = object_pose.position.z
	obj_angle = object_pose.rotation.angle_z.radians

	ref_x = reference_frame_pose.position.x
	ref_y = reference_frame_pose.position.y
	ref_angle = reference_frame_pose.rotation.angle_z.radians


	# 1. transpose to convert the object coordinates to reference coordinates without rotation
	obj_x_t = obj_x - ref_x
	obj_y_t = obj_y - ref_y

	# 2. Perform rotation of transposed object coordinates on angle of reference coordinate system.
	obj_x_n  = math.cos(ref_angle)*obj_x_t + math.sin(ref_angle)*obj_y_t
	obj_y_n = -math.sin(ref_angle) * obj_x_t + math.cos(ref_angle) * obj_y_t

	# 3. Update angle of the object by just subtracting the angle of reference coordinate system
	obj_angle_n = obj_angle - ref_angle

	# 4. Create pose object
	new_obj_pose = cozmo.util.pose_z_angle(obj_x_n, obj_y_n, obj_z, radians(obj_angle_n))

	return new_obj_pose

def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
				print("================================================")
				#get_relative_pose(cube.pose, robot.pose)

		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
