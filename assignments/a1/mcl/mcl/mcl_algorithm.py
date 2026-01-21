from typing import List, Tuple
import math
import random


class MCLAlgorithm:
    """
    Algorithmic logic for Monte Carlo Localization.
    """

    def __init__(
        self,
        num_particles,
        motion_noise_trans,
        motion_noise_rot,
        particle_noise_trans,
        particle_noise_rot,
    ):
        self.num_particles = num_particles
        self.particles = []

        # Motion model noise
        self.motion_noise_trans = motion_noise_trans # Noise of translational motion
        self.motion_noise_rot = motion_noise_rot # Noise of rotational motion

        # Noise added to particles when resampling
        self.particle_noise_trans = particle_noise_trans 
        self.particle_noise_rot = particle_noise_rot

        # Map related variables
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0


    def motion_model_prediction(self, v, w, dt):
        """
        Update every particle position based on velocity (v, w) and time (dt).

        Steps:
        1. Add noise to the control
        2. For each particle, calculate where it would move to given the noisy control 
           and assumed system dynamics

        Args:
            v (float): Linear velocity (meters/sec)
            w (float): Angular velocity (radians/sec)
            dt (float): Time interval since last update (seconds)
        """
        # STUDENT CODE START
        # STUDENT CODE END

    def sensor_model_update(self, observations: List[Tuple[float, float]]):
        """
        Update particle weights based on sensor data (Likelihood Field).

        Steps:
        1. For each particle, go through each observation
        2. Evaluate the likelihoods of observations using the map to obtain the weights
           of each particle
        3. Update the weights of each particle

        Args:
            observation (List[Tuple[float, float]]): 
                The valid [(range, angle), ...] readings from the LaserScan.
        """
        # Steps 1-3.
        # STUDENT CODE START
        # STUDENT CODE END

    def resample(self):
        """
        Replaces self.particles with a new set of particles drawn based on their 
        weights.

        Steps:
        1. Sanity check
        2. Draw self.num_particles samples from self.particles, with each of the 
           particles having probability proportional to its weight
        3. To each particle, add noise based on particle_noise_trans and 
           partical_noise_rot
        """
        # 1. Sanity check
        weights = [p[3] for p in self.particles]

        # Edge case: if all weights are zero (lost), return or re-init
        if sum(weights) == 0:
            return

        # 2. Weighted resampling
        # STUDENT CODE START
        # STUDENT CODE END

        # 3. Add noise to particles
        # STUDENT CODE START
        # STUDENT CODE END

    # =======================
    # Helpers: Do not modify!
    # =======================
    def normalize_weights(self):
        """Normalizes weights so they sum to 1.0"""
        total = sum(p[3] for p in self.particles)
        if total == 0:
            for p in self.particles:
                p[3] = 1.0 / self.num_particles
            return
        
        for p in self.particles:
            p[3] /= total

    def world_to_map(self, wx, wy):
        """
        Helper: Converts world coordinates to map index and returns occupancy value.
        """
        if self.map_data is None:
            return -1
        
        mx = int((wx - self.map_origin[0]) / self.map_resolution)
        my = int((wy - self.map_origin[1]) / self.map_resolution)
        
        if mx < 0 or mx >= self.map_width or my < 0 or my >= self.map_height:
            return -1
        
        return self.map_data[my * self.map_width + mx]


    def set_map(self, data, resolution, origin, width, height):
        """
        Helper to update map data from the node.
        """
        self.map_data = data
        self.map_resolution = resolution
        self.map_origin = origin
        self.map_width = width
        self.map_height = height