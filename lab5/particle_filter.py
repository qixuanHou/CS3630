# Author 1: Ramamurthy Siripuram
# Author 2: Daniel Ocano

from grid import *
from particle import Particle
from utils import *
from setting import *
import math
import numpy as np



def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    # print(type(particles[0]))
    # dx, dy, dh = add_odometry_noise(odom, 0.01, 0.01)
    # robot_heading
    dx, dy, dh = odom
    motion_particles = []
    for particle in particles:
        x, y, h = particle.xyh
        nh = add_gaussian_noise(h+dh, ODOM_HEAD_SIGMA)
        wx, wy = rotate_point(add_gaussian_noise(dx, ODOM_TRANS_SIGMA), add_gaussian_noise(dy, ODOM_TRANS_SIGMA), nh)

        motion_particles.append(Particle(x+wx, y+wy, nh))
    return motion_particles

------------------------------------------------------------------------
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

    def closest_marker(cm, mvp):
        # print("cm", cm)
        # print("mvp", mvp)
        return sorted([(grid_distance(cm[0], cm[1], m[0], m[1]), m) for m in mvp])[0]

    particles_off_grid = 0
    particles_weights = list()
    for particle in particles:
        if grid.is_free(particle.xy[0], particle.xy[1]):
            if len(measured_marker_list) > 0:
                markers_visible_to_particle = particle.read_markers(grid)
                if len(markers_visible_to_particle) > 0:
                    matched_markers = list()
                    for cm in measured_marker_list:
                        if len(markers_visible_to_particle) > 0:
                            m = closest_marker(cm, markers_visible_to_particle)[1]
                            matched_markers.append((cm, m))
                            markers_visible_to_particle.remove(m)
                    prob = 1.0
                    for marker in matched_markers:
                        distBetweenMarkers = grid_distance(marker[0][0], marker[0][1], marker[1][0], marker[1][1])
                        angleBetweenMarkers = diff_heading_deg(marker[0][2], marker[1][2])
                        a = ((distBetweenMarkers)**2) / (2*(MARKER_TRANS_SIGMA**2))
                        b = ((angleBetweenMarkers)**2) / (2*(MARKER_ROT_SIGMA**2))
                        prob *= math.exp(-(a+b))
                    particles_weights.append(prob)
                else:
                    particles_weights.append(0)
            else:
                particles_weights.append(1)
        else:
            particles_off_grid += 1
            particles_weights.append(0)

    # normalize
    # print(particles_weights, "\nlen", len(particles_weights))
    alpha = sum(particles_weights)
    # print("sumofw", sum(particles_weights))
    if alpha == 0:
        particles_weights = [1/(len(particles_weights)) for pw in particles_weights]
    else:
        particles_weights = [pw / alpha for pw in particles_weights]

    # particles_weights = [pw / alpha for pw in particles_weights]

    # print("sumofnw", sum(particles_weights))


    # print("particles_off_grid", particles_off_grid)
    # # resample
    # print(particles_weights.count(0))
    num_random_samples = 100
    # print(num_random_samples)
    random_particles = list()
    for x in range(num_random_samples):
        x, y = grid.random_free_place()
        random_particles.append(Particle(x, y))

    measured_particles = np.random.choice(particles, size=len(particles) - num_random_samples, p=particles_weights)

    return np.concatenate([measured_particles, random_particles])
