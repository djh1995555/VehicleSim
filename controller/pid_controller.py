#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from proto.controller_pb2 import *
class PIDController():
    def __init__(self, config, vehicle_param,sample_time):
        self._config = config
        self._sample_time = sample_time
        self._vehicle_param = vehicle_param

    def compute_control_cmd(self, state, trajectory, control_output, nearest_id):
        preview_length = int(self._config['preview_time'] / self._sample_time)
        preview_id = min(nearest_id + preview_length, len(trajectory.trajectory_points)-1)
        preview_point = trajectory.trajectory_points[preview_id]
        current_v = state.v
        error = preview_point.v - current_v

        control_output.control_debug.lon_controller_type = PID
        return error * self._config['propotional_gain']