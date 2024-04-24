#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
import yaml
import os
from utils.utils import *
from proto.controller_pb2 import *
from controller.pid_controller import PIDController
from controller.pure_pursuit_controller import PurePursuitController
from controller.kls_controller import KLSController
from dynamic_model.vehicle_dynamic_model import VehicleDynamicModel
from proto.vehicle_state_pb2 import *

class VehicleController():
    def __init__(self, simulator_config):
        with open(os.path.join(simulator_config['root_dir'],simulator_config['controller_config']), 'r') as f:
            self._config = yaml.load(f,Loader=yaml.Loader)

        with open(os.path.join(simulator_config['root_dir'],simulator_config['vehicle_model_config']), 'r') as f:
            self._vehicle_model_config = yaml.load(f,Loader=yaml.Loader)

        self._longitudinal_controller = PIDController(self._config['PID_param'], self._vehicle_model_config['vehicle_param'], simulator_config['sample_time'])
        if(self._config['controller_config']['lat_controller_type'] == "pp"):
            self._lateral_controller = PurePursuitController(self._config['PP_param'], self._vehicle_model_config['vehicle_param'], simulator_config['sample_time'])
        elif(self._config['controller_config']['lat_controller_type'] == "kls"):
            self._lateral_controller = KLSController(self._config['KLS_param'], self._vehicle_model_config['vehicle_param'], simulator_config['sample_time'])
        
        self._simulator_config = simulator_config
        self._cur_segment_id = 0
        self._match_start_id = 0
        self._is_stop_stage = False
        self._get_new_segment = False

    def use_brake(self, target_acc):
        return target_acc < 0
    
    def compute_torque(self, target_acc):
        return target_acc * 100
    
    def compute_brake(self, target_acc):
        return target_acc * 10

    def compute_steering_wheel_angle(self, front_angle):
        steer_ratio = 100
        return front_angle * steer_ratio * 57.3
    
    def compute_control_cmd(self, state, trajectory_segment, control_output):
        self._get_new_segment = self._cur_segment_id != trajectory_segment.segment_id
        self._cur_segment_id = trajectory_segment.segment_id
        nearest_id = self.match_point(state, trajectory_segment, control_output)

        cur_end_point = trajectory_segment.trajectory_points[-1]
        dx = cur_end_point.x - state.x    
        dy = cur_end_point.y - state.y
        distance = math.hypot(dx,dy)
        cur_heading = state.heading
        if(state.gear_state == GEAR_STATE_R):
            cur_heading += math.pi
        alpha = normalize_angle(math.atan2(dy, dx) - cur_heading)
        # not reach, distance < 0
        if(math.fabs(alpha) < math.pi / 2):
            distance *= -1
        control_output.control_debug.stop_distance = distance
        stop_acc = self._config['controller_config']['stop_acc']
        acc_response_delay = self._config['controller_config']['acc_response_delay']
        stop_distance_threshold = state.v * acc_response_delay + state.v * state.v / (2 * stop_acc)
        control_output.control_debug.stop_distance_threshold = -stop_distance_threshold
        if(distance > -stop_distance_threshold):
            self._is_stop_stage = True
        
        if(self._get_new_segment):
            self._is_stop_stage = False

        control_output.control_debug.get_new_segment = self._get_new_segment

        if(self._is_stop_stage):
            if(distance < 0):
                target_acc = - state.v * state.v / 2 / math.fabs(distance)
            if(distance >= 0):
                target_acc = -stop_acc
        else:
            target_acc = self._longitudinal_controller.compute_control_cmd(state, trajectory_segment, control_output, nearest_id)
        
        control_output.control_debug.is_stop_stage = self._is_stop_stage

        control_output.control_cmd.lon_cmd_type = ACC
        if(self._config['controller_config']['lon_cmd_type'] != "acc"):
            if(self.use_brake(target_acc)):
                control_output.control_cmd.target_brake = self.compute_brake(target_acc)
                control_output.control_cmd.lon_cmd_type = BRAKE
            else:
                control_output.control_cmd.target_torque = self.compute_torque(target_acc)
                control_output.control_cmd.lon_cmd_type = TORQUE
        control_output.control_cmd.target_acc = target_acc


        target_front_angle = self._lateral_controller.compute_control_cmd(state, trajectory_segment, control_output, nearest_id)
        control_output.control_cmd.lat_cmd_type = FRONT_ANGLE
        if(self._config['controller_config']['lat_cmd_type'] != "front_angle"):
            control_output.control_cmd.target_steering_angle = self.compute_steering_wheel_angle(target_front_angle)
            control_output.control_cmd.lat_cmd_type = STEERING_WHEEL_ANGLE        
        control_output.control_cmd.target_front_angle = target_front_angle
        
        target_state = trajectory_segment.trajectory_points[nearest_id]
        control_output.control_cmd.target_gear = target_state.gear_state

        
        control_output.control_debug.nearest_point_id = nearest_id
        control_output.control_debug.nearest_point_v = target_state.v
        control_output.control_debug.nearest_point_acc = target_state.acc
        control_output.control_debug.nearest_point_heading = target_state.heading
        control_output.control_debug.nearest_point_curvature = target_state.curvature


        control_output.control_debug.v_error = target_state.v - state.v
        control_output.control_debug.lat_error = compute_lateral_error(target_state, state)
        control_output.control_debug.heading_error = state.heading - target_state.heading

    def compute_distance(self, target_point, state):
        dx = target_point.x - state.x    
        dy = target_point.y - state.y
        distance = math.hypot(dx,dy)
        return distance

    def match_point(self, state, trajectory_segment, control_output):
        if(self._get_new_segment):
            self._match_start_id = 1

        nearest_id = self._match_start_id
        max_distance =  math.inf
        for i in range(self._match_start_id, min(self._match_start_id + 50, len(trajectory_segment.trajectory_points))):
            trajectory_segment_point = trajectory_segment.trajectory_points[i]
            distance = self.compute_distance(trajectory_segment_point, state) 
            if(distance <= max_distance):
                max_distance = distance
                nearest_id = i       
        self._match_start_id = nearest_id
        return self._match_start_id