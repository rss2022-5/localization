#!/usr/bin/env python2

import numpy as np
import rospy

class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.
        self.MAX_PARTICLES     = int(rospy.get_param("~num_particles"))
        self.local_deltas      = np.zeros((self.MAX_PARTICLES, 3))

        self.deterministic     = rospy.get_param("~deterministic")

        ####################################

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """

        ####################################
        cosines = np.cos(particles[:,2])
        sines = np.sin(particles[:,2])

        self.local_deltas[:,0] = cosines*odometry[0] - sines*odometry[1]
        self.local_deltas[:,1] = sines*odometry[0] + cosines*odometry[1]
        self.local_deltas[:,2] = odometry[2]

        particles[:,:] += self.local_deltas
        N = particles.shape[0]

        if not self.deterministic:
            particles[:,0] += np.random.normal(loc=0.0,scale=0.05,size=N)
            particles[:,1] += np.random.normal(loc=0.0,scale=0.02,size=N)
            particles[:,2] += np.random.normal(loc=0.0,scale=0.05,size=N)

        return particles
        ####################################
