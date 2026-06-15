# -*- coding: utf-8 -*-

import math


def compute_rotations_and_mod(angle_deg):
    """返回 (总圈数, 模360角度) 对于正负数均正确"""
    if angle_deg == 0:
        return 0, 0.0
    total_rot = math.floor(angle_deg / 360.0)
    mod_angle = angle_deg % 360.0
    return total_rot, mod_angle