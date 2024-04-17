#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
import yaml
import os
from proto.controller_pb2 import ControlCommand
from controller.pid_controller import PIDController
from controller.pure_pursuit_controller import PurePursuitController
from dynamic_model.vehicle_dynamic_model import VehicleDynamicModel

class VehicleController():
    def __init__(self, simulator_config):
        with open(os.path.join(simulator_config['root_dir'],simulator_config['controller_config']), 'r') as f:
            self._controller_config = yaml.load(f,Loader=yaml.Loader)

        with open(os.path.join(simulator_config['root_dir'],simulator_config['vehicle_model_config']), 'r') as f:
            self._vehicle_model_config = yaml.load(f,Loader=yaml.Loader)

        self._longitudinal_controller = PIDController(self._controller_config['PID_param'], self._vehicle_model_config['vehicle_param'], simulator_config['sample_time'])
        self._lateral_controller = PurePursuitController(self._controller_config['PP_param'], self._vehicle_model_config['vehicle_param'], simulator_config['sample_time'])

        self._simulator_config = simulator_config
        self._match_start_id = 0

    def match_point(self, state, trajectory):
        target_id = self._match_start_id
        max_distance =  math.inf
        for i in range(self._match_start_id, len(trajectory.trajectory_points)):
            trajectory_point = trajectory.trajectory_points[i]
            dx = trajectory_point.x - state.x    
            dy = trajectory_point.y - state.y
            distance = math.hypot(dx,dy)
            if(distance < max_distance):
                max_distance = distance
                target_id = i
        
        self._match_start_id = target_id
        return self._match_start_id

    def compute_control_cmd(self, state, trajectory, control_output):
        target_id = self.match_point(state, trajectory)
        control_output.control_cmd.target_acc = self._longitudinal_controller.compute_control_cmd(state, trajectory, control_output, target_id)
        control_output.control_cmd.target_front_angle = self._lateral_controller.compute_control_cmd(state, trajectory, control_output, target_id)

        control_output.control_debug.nearest_point_id = target_id
        control_output.control_debug.nearest_point_v = trajectory.trajectory_points[target_id].v
        control_output.control_debug.nearest_point_acc = trajectory.trajectory_points[target_id].acc
        control_output.control_debug.nearest_point_heading = trajectory.trajectory_points[target_id].heading
        control_output.control_debug.nearest_point_curvature = trajectory.trajectory_points[target_id].curvature