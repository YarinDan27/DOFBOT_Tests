#!/usr/bin/env python3
"""
The convolutional neural network (CNN). (Milestone M4 -- we build this together.)

Defined once here and imported by BOTH learn/train.py (laptop) and
vision/classifier.py (Jetson), so the two can never drift apart.

Job: one square picture in -> one of models/<version>/labels.json out.
Start with 3 labels: empty / white / black. That's enough to play, because
the game history says which piece is which. Piece types (13 labels) later.

Plan for round 1 (from scratch, to learn how it works):
    crop (3 colour channels x H x W)
      -> [conv 3x3 -> ReLU -> max-pool] x 3   (learns edges, then shapes, then "piece-ness")
      -> flatten -> fully connected -> 3 scores -> softmax = confidence per label
Round 2: swap in a small pretrained network (transfer learning) and compare.
"""


def build(num_labels):
    raise NotImplementedError("CNN not built yet (milestone M4)")
