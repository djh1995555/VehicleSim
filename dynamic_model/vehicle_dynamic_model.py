#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
import queue
import os
import yaml
from proto.vehicle_state_pb2 import VehicleState

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
        self.reset(self._simulator_config['init_v'],self._simulator_config['init_pitch'])
        

    def reset(self, init_v = 0.0, init_pitch = 0.0):
        self._vehicle_state = VehicleState()

        self._vehicle_state.v = init_v
        self._vehicle_state.pitch = init_pitch
        self._signal_buffer.reset()
    
    def update_longitudinal_state(self, acc):
        valid_acc = self._signal_buffer.after_buffer('acc', acc, input=True)
            
        self._vehicle_state.v += valid_acc * self._sample_time
        self._vehicle_state.v = min(30,self._vehicle_state.v)
        self._vehicle_state.acc = valid_acc

    def update_lateral_state(self, front_angle):
        v = self._vehicle_state.v

        valid_front_angle = self._signal_buffer.after_buffer('front_angle', front_angle, input=True)
            
        self._vehicle_state.curvature = math.tanh(valid_front_angle) / self._config['vehicle_param']['wheel_base']
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
