#!/usr/bin/env python
import math

def compute_lateral_error(target_point, state):
    dx = state.x - target_point.x
    dy = state.y - target_point.y
    heading = state.heading
    return dy * math.sin(heading) + (-dx * math.cos(heading))

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle