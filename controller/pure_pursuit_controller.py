#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from proto.controller_pb2 import *

class PurePursuitController():
    def __init__(self, config, vehicle_param, sample_time):
        self._config = config
        self._sample_time = sample_time
        self._vehicle_param = vehicle_param

    def compute_control_cmd(self, state, trajectory, control_output, nearest_id):
        preview_length = int(self._config['preview_time'] / self._sample_time)
        preview_id = min(nearest_id + preview_length, len(trajectory.trajectory_points)-1)
        preview_point = trajectory.trajectory_points[preview_id]

        dx = preview_point.x - state.x    
        dy = preview_point.y - state.y
        distance = math.hypot(dx,dy)
        alpha = math.atan2(dy, dx) - state.heading
        delta = math.atan2(2.0 * self._vehicle_param['wheel_base'] * math.sin(alpha) / distance, 1.0)

        control_output.control_debug.lat_controller_type = PP
        return delta