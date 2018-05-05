from grid import *
from particle import Particle
from utils import *
from setting import *
import math
import numpy as np

def normalize_angle(h):
    while h > 180:
        h -= 360
    while h <= -180:
        h += 360
    return h

# Just moves the particle using given odometry reading
def move_particle(particle, odom):
    x, y, h = particle.xyh
    dx, dy = rotate_point(odom[0], odom[1], h)
    new_x = x + dx
    new_y = y + dy
    new_h = h + odom[2]
    new_h = normalize_angle(new_h)
    new_particle = Particle(new_x, new_y, new_h)
    return new_particle

def eucledian_distance(m1, m2):
    x1 = m1[0]
    y1 = m1[1]
    x2 = m2[0]
    y2 = m2[1]
    dist = grid_distance(x1, y1, x2, y2)
    return dist

# Converts the distance between sensor marker position and particle marker position to the weight
# Uses the function that is 1 when distance is 0 and decays to 0 as distance growth.
def distance_to_probability(distance):
    alpha = 3.0
    prob = -2.0/(1.0+alpha**(-distance)) + 2.0
    return prob

# Converts the heading difference between sensor marker position and particle marker position to the weight
# Uses linear function that is 1 at 0 and 0 at 45, the values that are higher than 45 get weight as 0
# Assuming that heading is in degrees in [0, 180]
def heading_to_probability(heading):
    if heading <0 or heading > 180:
        raise Exception("Bad heading")
    #Just a linear function
    value = heading*(-1./45.0)+1
    if value < 0:
        value = 0
    return value

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """

    motion_particles = []

    # For each particle, take the odometry reading, add noise, and just move the particle
    for i in range(len(particles)):
        particle = particles[i]
        odom_with_noise = add_odometry_noise(odom, ODOM_HEAD_SIGMA, ODOM_TRANS_SIGMA)
        motion_particles.append(move_particle(particle, odom_with_noise))
        #motion_particles.append(move_particle(particle, odom))

    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update


    """

    # calculate weights
    heading_impact = 0.3

    weights = []
    for i in range(len(particles)):
        particle = particles[i]
        particle_markers = particle.read_markers(grid)
        particle_prob = 0



        # If sensors do not read anything and particles as well, just assign the max weight, if particle reads something, but sensors do not, asssign 0
        if len(measured_marker_list) == 0:
            if len(particle_markers) > 0:
                particle_prob = 0
            else:
                particle_prob = 1 + heading_impact*1.0

        if len(measured_marker_list) > 0:
            for p_i in range(len(particle_markers)):
                particle_marker = particle_markers[p_i]
                particle_marker = add_marker_measurement_noise(particle_marker, MARKER_TRANS_SIGMA, MARKER_ROT_SIGMA)
                shortest_distance = 100000
                best_marker = None
                # For each particle marker reading, find closest reading from the sensors
                for m_i in range(len(measured_marker_list)):
                    marker = measured_marker_list[m_i]
                    distance = eucledian_distance(marker, particle_marker)
                    if distance < shortest_distance:
                        shortest_distance = distance
                        best_marker = marker

                # Find dif in heading calculate heading weight and distance weight
                heading_diff = abs(diff_heading_deg(best_marker[2], particle_marker[2]))
                prob_dist = distance_to_probability(shortest_distance)
                prob_heading = heading_to_probability(heading_diff)

                # Get total weight
                prob = 1.0 * prob_dist + heading_impact * prob_heading
                # print("Distance: " + str(distance))
                # print("Heading: " + str(heading_diff))
                # print("Heading weight: " + str(prob_heading))
                # print("Distance weight: " + str(prob_dist))
                # print("Total weight: " + str(prob))
                # print("===============")

                particle_prob += prob

        weights.append(particle_prob)

    # normalize weights
    weights_sum = sum(weights)
    if weights_sum == 0:
        return particles

    weights = [w/weights_sum for w in weights]

    # resample
    total_particles = len(particles)
    # Add .1% of some random particles
    random_particles_count = int(total_particles * 0.001)
    resampling_size = total_particles - random_particles_count
    measured_particles = np.random.choice(particles, size = resampling_size, p = weights).tolist()
    measured_particles+=Particle.create_random(random_particles_count, grid)

    #print(measured_particles)

    return measured_particles


