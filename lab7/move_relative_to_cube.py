#!/usr/bin/env python3

'''
This is starter code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import sys

from cozmo.util import degrees
from cozmo.util import radians
from odometry import cozmo_go_to_pose
sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose
import odometry

def get_global_pose_from_local(ref_pose, local_pose):
	local_x = local_pose.position.x
	local_y = local_pose.position.y
	local_z = local_pose.position.z
	local_angle = local_pose.rotation.angle_z.radians

	ref_x = ref_pose.position.x
	ref_y = ref_pose.position.y
	ref_angle = ref_pose.rotation.angle_z.radians

	# 1. translate to convert the object coordinates to reference coordinates without rotation
	local_x_t = local_x + ref_x
	local_y_t = local_y + ref_y

	# 2. Perform rotation of transposed object coordinates on angle of reference coordinate system.
	local_x_n = math.cos(ref_angle) * local_x_t - math.sin(ref_angle) * local_y_t
	local_y_n = math.sin(ref_angle) * local_x_t + math.cos(ref_angle) * local_y_t

	# 3. Update angle of the object by just subtracting the angle of reference coordinate system
	local_angle_n = local_angle + ref_angle

	# 4. Create pose object
	new_obj_pose = cozmo.util.pose_z_angle(local_x_n, local_y_n, local_z, radians(local_angle_n))

	return new_obj_pose

def move_relative_to_cube(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, when a cube is detected it 
	moves the robot to a given pose relative to the detected cube pose.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while cube is None:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Found a cube, pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")

	print("Cube angle: " + str(cube.pose.rotation.angle_z.degrees))
	desired_pose_relative_to_cube = Pose(0, 30, 0, angle_z=degrees(-90))

	desired_global_pose = get_global_pose_from_local(cube.pose, desired_pose_relative_to_cube)
	print("Cube pose: %s" % cube.pose)
	print("Desired global pose: %s" % desired_global_pose)
	relative_pose = get_relative_pose(desired_global_pose, robot.pose)
	x = relative_pose.position.x
	y = relative_pose.position.y
	angle = relative_pose.rotation.angle_z.degrees
	print(x)
	print(y)
	print(angle)
	odometry.my_go_to_pose1(robot, x, y, angle)

	# ####
	# TODO: Make the robot move to the given desired_pose_relative_to_cube.
	# Use the get_relative_pose function your implemented to determine the
	# desired robot pose relative to the robot's current pose and then use
	# one of the go_to_pose functions you implemented in Lab 6.
	# ####




if __name__ == '__main__':

	cozmo.run_program(move_relative_to_cube)
