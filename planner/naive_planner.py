#!/usr/bin/env python
import numpy as np
import pandas as pd
import math
import yaml
import os
from proto.trajectory_pb2 import Trajectory

from proto.vehicle_state_pb2 import VehicleState

class KinematicDynamicModel(object):
    def __init__(self, simulator_config):
        config_filepath = os.path.join(simulator_config['root_dir'],simulator_config['vehicle_model_config'])
        with open(config_filepath, 'r') as f:
            self._config = yaml.load(f,Loader=yaml.Loader)
        self._simulator_config = simulator_config
        self._sample_time = simulator_config['sample_time']
        self._vehicle_state = VehicleState()
        self.reset(self._simulator_config['init_v'],self._simulator_config['init_pitch'])

    def reset(self, init_v = 0.0, init_pitch = 0.0):
        self._vehicle_state = VehicleState()
        self._vehicle_state.v = init_v
        self._vehicle_state.pitch = init_pitch
    
    def update_longitudinal_state(self, acc):
        self._vehicle_state.v += acc * self._sample_time
        self._vehicle_state.v = min(30,self._vehicle_state.v)
        self._vehicle_state.acc = acc

    def update_lateral_state(self, front_angle):
        v = self._vehicle_state.v
            
        self._vehicle_state.curvature = math.tanh(front_angle) / self._config['vehicle_param']['wheel_base']
        heading_rate = v * self._vehicle_state.curvature
        self._vehicle_state.heading += heading_rate * self._sample_time
        self._vehicle_state.heading_rate = heading_rate
        self._vehicle_state.x += v * math.cos(self._vehicle_state.heading) * self._sample_time
        self._vehicle_state.y += v * math.sin(self._vehicle_state.heading) * self._sample_time
        

    def update(self, acc, front_angle):
        self.update_longitudinal_state(acc)
        self.update_lateral_state(front_angle)

    def get_state(self):
        return self._vehicle_state
    
class NaivePlanner():
    def __init__(self, simulator_config):
        config_filepath = os.path.join(simulator_config['root_dir'],simulator_config['naive_planner_config'])
        with open(config_filepath, 'r') as f:
            self._config = yaml.load(f,Loader=yaml.Loader)
        
        self._simulator_config = simulator_config
        self._kinematic_dyanmaic_model = KinematicDynamicModel(simulator_config)

    def set_point(self, point, vehicle_state):
        point.x = vehicle_state.x
        point.y = vehicle_state.y
        point.heading = vehicle_state.heading
        point.v = vehicle_state.v
        point.acc = vehicle_state.acc
        point.curvature = vehicle_state.curvature
    
    def generate_trajectory(self):
        self._total_length = int(self._simulator_config['simulation_time'] / self._simulator_config['sample_time'])
        t = [self._simulator_config['sample_time'] * i for i in range(self._total_length)]
        front_angle_list = 0.05 * np.sin(np.dot(2 * math.pi / 100, t))
        acc_list = 0.2 * np.sin(np.dot(2 * math.pi / 100, t)) + 0.02

        trajectory = Trajectory()
        init_state = self._kinematic_dyanmaic_model.get_state()
        init_point = trajectory.trajectory_points.add()
        self.set_point(init_point, init_state)
        for front_angle, acc in zip(front_angle_list, acc_list):
            self._kinematic_dyanmaic_model.update(acc, front_angle)
            state = self._kinematic_dyanmaic_model.get_state()
            point = trajectory.trajectory_points.add()
            self.set_point(point, state)
        
        return trajectory




        




