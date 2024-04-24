#!/usr/bin/env python
import numpy as np
import pandas as pd
import math
import yaml
import os
from utils.utils import *
from proto.trajectory_pb2 import *
from proto.vehicle_state_pb2 import *

class KinematicDynamicModel(object):
    def __init__(self, simulator_config):
        config_filepath = os.path.join(simulator_config['root_dir'],simulator_config['vehicle_model_config'])
        with open(config_filepath, 'r') as f:
            self._config = yaml.load(f,Loader=yaml.Loader)
        self._simulator_config = simulator_config
        self._sample_time = simulator_config['sample_time']
        self._vehicle_state = VehicleState()
        

    def reset(self, init_v = 0.0, init_pitch = 0.0):
        self._vehicle_state = VehicleState()
        self._vehicle_state.v = init_v
        self._vehicle_state.pitch = init_pitch
    
    def update_longitudinal_state(self, acc, gear):
        self._vehicle_state.v += acc * self._sample_time
        self._vehicle_state.v = min(30,self._vehicle_state.v)
        self._vehicle_state.acc = acc

    def update_lateral_state(self, front_angle, gear):
        if(gear == GEAR_STATE_R):
            v = - self._vehicle_state.v 
        else:
            v = self._vehicle_state.v 
            
        self._vehicle_state.curvature = math.tanh(front_angle) / self._config['vehicle_param']['wheel_base']
        heading_rate = v * self._vehicle_state.curvature
        self._vehicle_state.heading += heading_rate * self._sample_time
        self._vehicle_state.heading_rate = heading_rate
        self._vehicle_state.x += v * math.cos(self._vehicle_state.heading) * self._sample_time
        self._vehicle_state.y += v * math.sin(self._vehicle_state.heading) * self._sample_time
        self._vehicle_state.gear_state = gear

    def update(self, acc, front_angle, gear):
        self.update_longitudinal_state(acc, gear)
        self.update_lateral_state(front_angle, gear)

    def get_state(self):
        return self._vehicle_state
    
    def get_wheel_base(self):
        return self._config['vehicle_param']['wheel_base']
    
class NaivePlanner():
    def __init__(self, simulator_config):
        config_filepath = os.path.join(simulator_config['root_dir'],simulator_config['naive_planner_config'])
        with open(config_filepath, 'r') as f:
            self._config = yaml.load(f,Loader=yaml.Loader)
        
        self._simulator_config = simulator_config
        self._kinematic_dyanmaic_model = KinematicDynamicModel(simulator_config)
        if(self._simulator_config['task_type']=='noa'):
            traj_config = self._config['noa_trajectory']
        elif(self._simulator_config['task_type']=='apa'):
            traj_config = self._config['apa_trajectory']
        self._kinematic_dyanmaic_model.reset(traj_config['init_v'],traj_config['init_pitch'])
        self._segment_id = 0

    def set_point(self, point, vehicle_state):
        point.x = vehicle_state.x
        point.y = vehicle_state.y
        point.heading = vehicle_state.heading
        point.v = vehicle_state.v
        point.acc = vehicle_state.acc
        point.curvature = vehicle_state.curvature
        point.gear_state = vehicle_state.gear_state
    
    def get_segment(self, state):
        cur_segment = self._trajectory.trajectory_segments[self._segment_id]
        cur_end_point = cur_segment.trajectory_points[-1]
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

        is_finish = False
        reach_tolerance = -0.05
        if(distance > reach_tolerance and state.v == 0):
            if(self._segment_id == self._trajectory.segment_num - 1):
                is_finish = True
            else:
                self._segment_id += 1
        return is_finish, self._trajectory.trajectory_segments[self._segment_id]    

    def generate_trajectory(self):
        self._total_length = int(self._simulator_config['simulation_time'] / self._simulator_config['sample_time'])
        t = [self._simulator_config['sample_time'] * i for i in range(self._total_length)]

        trajectory = Trajectory()
        segment_list = []
        if(self._simulator_config['task_type']=='noa'):
            trajectory.task_type = NOA
            self._segment_num = self._config['noa_trajectory']['segment_num']
            front_angle_list = 0.01 * np.sin(np.dot(2 * math.pi / 100, t))
            acc_list = 0.2 * np.sin(np.dot(2 * math.pi / 100, t)) + 0.02
            gear_list = [GEAR_STATE_D] * self._total_length
            segment_list.append((front_angle_list, acc_list, gear_list))
        elif(self._simulator_config['task_type']=='apa'):
            trajectory.task_type = APA
            self._segment_num = self._config['apa_trajectory']['segment_num']
            arc_ratio = self._config['apa_trajectory']['arc_ratio']

            wheel_base = self._kinematic_dyanmaic_model.get_wheel_base()
            init_v = self._config['apa_trajectory']['init_v']
            target_front_angle = math.atan2(2 * math.pi * arc_ratio * wheel_base / (init_v * self._simulator_config['simulation_time']), 1.0)
            segment_length = int(self._total_length / self._segment_num)

            direction = 1
            cur_length = 0
            for i in range(self._segment_num - 1):
                front_angle_list = [direction * target_front_angle] * segment_length
                acc_list = [0] * segment_length
                if(direction == 1):
                    gear_list = [GEAR_STATE_D] * segment_length
                else:
                    gear_list = [GEAR_STATE_R] * segment_length
                direction *= -1
                segment_list.append((front_angle_list, acc_list, gear_list))
                cur_length += segment_length

            end_length = self._total_length - cur_length
            front_angle_list = [direction * target_front_angle] * end_length
            acc_list = [0] * end_length
            if(direction == 1):
                gear_list = [GEAR_STATE_D] * end_length
            else:
                gear_list = [GEAR_STATE_R] * end_length
            segment_list.append((front_angle_list, acc_list, gear_list))   

        trajectory.segment_num = self._segment_num
        
        segment_id = 0
        for front_angle_list, acc_list, gear_list in segment_list:
            trajectory_segment = trajectory.trajectory_segments.add()
            trajectory_segment.segment_id = segment_id
            for front_angle, acc, gear in zip(front_angle_list, acc_list, gear_list):
                state = self._kinematic_dyanmaic_model.get_state()
                point = trajectory_segment.trajectory_points.add()
                self.set_point(point, state)
                self._kinematic_dyanmaic_model.update(acc, front_angle, gear)
            segment_id += 1
        
        self._trajectory = trajectory
        return trajectory




        