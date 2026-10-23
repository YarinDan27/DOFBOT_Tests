#!/usr/bin/env python3
"""
Grades a trained model on the held-out test sessions. (Milestone M4.)

Two scores, because one isn't enough:
  * confusion matrix -- which labels get mixed up (e.g. black on dark squares)
  * whole-board accuracy -- the share of photos where ALL 64 squares are right.
    One wrong square = a wrong move, so this is the number that matters.
"""


def evaluate(model_dir):
    raise NotImplementedError("evaluation not built yet (milestone M4)")
