#!/usr/bin/env python3
"""
The convolutional neural network (CNN) that reads one square.

Defined once here and used by learn/train.py (on the GPU) -- the Jetson
runs the exported copy (models/vNNN/model.onnx) through vision/classifier.py,
so the two can't drift apart.

Job: one square picture in -> a score per label out. Round 1: 3 labels,
empty / white / black. That's enough to play: the game's history says which
piece is which. (Which piece -- 13 labels -- can come later.)

    picture (3 colours x 96 tall x 64 wide), pixel values 0-255, BGR
      -> scale to 0..1, subtract the mean, divide by the spread  (PREPROCESS)
      -> [conv 3x3 -> batch norm -> ReLU -> max-pool 2x2] x 3
            96x64 -> 48x32 -> 24x16 -> 12x8, with 16 -> 32 -> 64 channels:
            first edges, then shapes, then "a piece is here, and it's light"
      -> flatten -> dropout -> 64 -> ReLU -> one score per label
    softmax(scores) = how sure it is of each label

Only plain layers (no adaptive pooling), so OpenCV's dnn module on the
Jetson can run the exported ONNX file.
"""
LABELS = ["empty", "white", "black"]
INPUT_H, INPUT_W = 96, 64
# applied OUTSIDE the network (here, and in classifier.py from the manifest)
PREPROCESS = {"scale": 1.0 / 255.0, "mean": [0.45, 0.45, 0.45], "std": [0.25, 0.25, 0.25],
              "channels": "BGR", "height": INPUT_H, "width": INPUT_W}


def build(num_labels=len(LABELS)):
    import torch.nn as nn

    def block(cin, cout):
        return [nn.Conv2d(cin, cout, 3, padding=1), nn.BatchNorm2d(cout), nn.ReLU(),
                nn.MaxPool2d(2)]

    return nn.Sequential(
        *(block(3, 16) + block(16, 32) + block(32, 64)),
        nn.Flatten(),
        nn.Dropout(0.3),
        nn.Linear(64 * (INPUT_H // 8) * (INPUT_W // 8), 64),
        nn.ReLU(),
        nn.Linear(64, num_labels))


def to_input(images, pre=None):
    """
    Square pictures (H x W x 3 uint8, BGR) -> a float32 batch N x 3 x H x W,
    preprocessed the way the network was trained. numpy only (no torch), so
    the Jetson uses the same code.
    """
    import cv2
    import numpy as np
    pre = pre or PREPROCESS
    out = np.empty((len(images), 3, pre["height"], pre["width"]), np.float32)
    mean = np.array(pre["mean"], np.float32).reshape(3, 1, 1)
    std = np.array(pre["std"], np.float32).reshape(3, 1, 1)
    for i, img in enumerate(images):
        if img.shape[0] != pre["height"] or img.shape[1] != pre["width"]:
            img = cv2.resize(img, (pre["width"], pre["height"]), interpolation=cv2.INTER_AREA)
        x = img.astype(np.float32).transpose(2, 0, 1) * pre["scale"]
        out[i] = (x - mean) / std
    return out
