#!/usr/bin/env python3
import numpy as np
import config

class BoardMapper:
    def __init__(self):
        print("[Vision] Initializing Spatial Mapping Coordinates...")
        
    def get_square_joints(self, square_name, base_joints):
        """Maps an algebraic square string (e.g. 'e4') into custom joint configurations."""
        # Interpolates layout positions relative to your validated grab base profile
        col = ord(square_name[0]) - ord('a')
        row = int(square_name[1]) - 1
        
        # Calculate localized angular deviations across your 8x8 matrix
        joints = list(base_joints)
        joints[0] = base_joints[0] + (col - 4) * 3  # S1 base sweep offset
        joints[1] = base_joints[1] + (row - 4) * 2  # S2 shoulder projection reach
        return joints
