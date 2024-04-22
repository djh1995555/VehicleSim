#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
import queue
import os
import yaml
from proto.vehicle_state_pb2 import VehicleState
from proto.controller_pb2 import *

class NoiseGenerator(object):
    def __init__(self, deviation):
        self._length = 1000
        self._noise = np.random.normal(0, deviation, size=(self._length,))
        self._idx = 0

    def get_noise(self):
        current_noise = self._noise[self._idx%self._length]
        self._idx += 1
        return current_noise
    
class SignalBuffer(object):
    def __init__(self, buffer_config, sample_time, for_simulator = True):
        self._buffer = {}
        self._buffer_config = buffer_config
        self._sample_time = sample_time
        self._for_simulator = for_simulator
    
    def after_buffer(self, signal_name, data, input = True):
        
        valid_data = 0.0
        if(input):
            query_name = "{}_input".format(signal_name)
        else:
            query_name = "{}_output".format(signal_name)

        if query_name not in self._buffer.keys():
            self._buffer[query_name] = queue.Queue()

        self._buffer[query_name].put(data)

        delay_frames = self._buffer_config[query_name] / self._sample_time

        if(self._buffer[query_name].qsize() >= int(delay_frames)):
            valid_data = self._buffer[query_name].get()

        return valid_data

    def reset(self):
        self._buffer = {}

class VehicleDynamicModel(object):
    def __init__(self, simulator_config):
        config_filepath = os.path.join(simulator_config['root_dir'],simulator_config['vehicle_real_config'])
        
        with open(config_filepath, 'r') as f:
            self._config = yaml.load(f,Loader=yaml.Loader)
        self._simulator_config = simulator_config
        self._sample_time = simulator_config['sample_time']
        self._vehicle_state = VehicleState()
        self._signal_buffer = SignalBuffer(self._config['buffer_config'], self._sample_time)
        

    def reset(self, init_v = 0.0, init_pitch = 0.0):
        self._vehicle_state = VehicleState()

        self._vehicle_state.v = init_v
        self._vehicle_state.pitch = init_pitch
        self._signal_buffer.reset()
    
    def compute_acc_from_torque(self, torque):
        return torque / 100

    def compute_acc_from_brake(self, brake):
        return brake / 100

    def compute_front_angle_from_steering_angle(self, steering_angle):
        steer_ratio = 100
        return steering_angle / steer_ratio / 57.3
         
    def update_longitudinal_state(self, control_cmd):
        if(control_cmd.lon_cmd_type == ACC):
            valid_acc = self._signal_buffer.after_buffer('acc', control_cmd.target_acc, input=True)
        elif(control_cmd.lon_cmd_type == TORQUE):
            valid_torque = self._signal_buffer.after_buffer('torque', control_cmd.target_torque, input=True)
            valid_acc = self.compute_acc_from_torque(valid_torque)
        elif(control_cmd.lon_cmd_type == BRAKE):
            valid_brake = self._signal_buffer.after_buffer('brake', control_cmd.target_brake, input=True)
            valid_acc = self.compute_acc_from_brake(valid_brake)
            
        self._vehicle_state.v += valid_acc * self._sample_time
        self._vehicle_state.v = min(30,self._vehicle_state.v)
        self._vehicle_state.acc = valid_acc

    def update_lateral_state(self, control_cmd):
        if(control_cmd.lat_cmd_type == FRONT_ANGLE):
            valid_front_angle = self._signal_buffer.after_buffer('front_angle', control_cmd.target_front_angle, input=True)
            self._vehicle_state.front_angle = valid_front_angle
        elif(control_cmd.lat_cmd_type == STEERING_WHEEL_ANGLE):
            valid_steering_angle = self._signal_buffer.after_buffer('steering_angle', control_cmd.target_steering_angle, input=True)
            valid_front_angle = self.compute_front_angle_from_steering_angle(valid_steering_angle)
            self._vehicle_state.steering_angle = valid_steering_angle
            self._vehicle_state.front_angle = valid_front_angle

        if(control_cmd.target_gear == GEAR_CMD_R):
            v = - self._vehicle_state.v 
        else:
            v = self._vehicle_state.v 
        self._vehicle_state.curvature = math.tanh(valid_front_angle) / self._config['vehicle_param']['wheel_base']
        heading_rate = v * self._vehicle_state.curvature
        self._vehicle_state.heading += heading_rate * self._sample_time
        self._vehicle_state.heading_rate = heading_rate
        self._vehicle_state.x += v * math.cos(self._vehicle_state.heading) * self._sample_time
        self._vehicle_state.y += v * math.sin(self._vehicle_state.heading) * self._sample_time
        self._vehicle_state.gear_state = control_cmd.target_gear
        

    def update(self, control_cmd):
        self.update_longitudinal_state(control_cmd)
        self.update_lateral_state(control_cmd)

    def get_state(self):
        return self._vehicle_state
