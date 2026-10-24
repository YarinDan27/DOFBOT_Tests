#!/usr/bin/env python3
"""
Runs a trained CNN on the Jetson. (Milestone M4 -- not built yet.)

Input:  the 64 crops from board_view.py, and a model folder
        (models/v001/: weights + labels.json + preprocess.json)
Output: {'a1': ('white', 0.99), ...} -- a label and a confidence per square.
        Low confidence anywhere = take another photo or ask, never guess.
"""


def classify(crops, model_dir):
    raise NotImplementedError("classifier not built yet (milestone M4)")
