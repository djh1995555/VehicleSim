#!/usr/bin/env python
import math
import numpy as np
import pandas as pd
from utils.utils import *
from proto.vehicle_state_pb2 import *
from proto.controller_pb2 import *

class SMController():
    def __init__(self, config, vehicle_param, sample_time):
        self._config = config
        self._sample_time = sample_time
        self._vehicle_param = vehicle_param

    def compute_control_cmd(self, state, trajectory, control_output, nearest_id):
        nearest_point = trajectory.trajectory_points[nearest_id]

        preview_length = int(self._config['preview_time'] / self._sample_time)
        preview_id = min(nearest_id + preview_length, len(trajectory.trajectory_points)-1)
        preview_point = trajectory.trajectory_points[preview_id]

        if(self._config['use_preview_lat_err']):
            lateral_error = compute_lateral_error(preview_point, state)
        else:
            lateral_error = compute_lateral_error(nearest_point, state)

        heading = state.heading
        # if(state.gear_state == GEAR_STATE_R):
        #     heading += math.pi
        # heading = normalize_angle(heading)
        if(self._config['use_preview_heading_err']):
            heading_error = heading - preview_point.heading
        else:
            heading_error = heading - nearest_point.heading
            
        if(state.gear_state == GEAR_STATE_R):
            heading_error *= -1

        lateral_error_rate = heading_error * state.v
        sigma = lateral_error_rate + self._config['eta_beta'] * sgn(lateral_error, self._config['epsilon_beta']) * math.sqrt(math.fabs(lateral_error))
        feedback_kappa = -self._config['eta_alpha'] * sgn(sigma, self._config['epsilon_alpha'])

        if(self._config['use_preview_curvature']):
            feedforward_kappa = preview_point.curvature * math.fabs(heading_error) / (1 - preview_point.curvature * lateral_error)
        else:
            feedforward_kappa = nearest_point.curvature * math.fabs(heading_error) / (1 - nearest_point.curvature * lateral_error)

        control_kappa = feedforward_kappa + feedback_kappa
        delta = math.atan2(control_kappa * self._vehicle_param['wheel_base'], 1.0)

        control_output.control_debug.lat_controller_type = SMC
        control_output.control_debug.control_lat_error = lateral_error
        control_output.control_debug.control_heading_error = heading_error
        control_output.control_debug.feedback_kappa = feedback_kappa
        control_output.control_debug.feedforward_kappa = feedforward_kappa
        control_output.control_debug.control_kappa = control_kappa

        return delta