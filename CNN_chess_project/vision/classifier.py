#!/usr/bin/env python3
"""
Runs a trained square CNN on the Jetson -- with OpenCV's dnn module, so the
Jetson needs no PyTorch.

Input:  the 64 crops from board_view.py, and a model folder
        (models/vNNN/: model.onnx + manifest.json + check.npz, made by
        learn/train.py on the GPU)
Output: {'a1': ('white', 0.99), ...} -- a label and how sure, per square.
        Not sure anywhere (below MIN_CONFIDENCE) = take another photo or ask;
        never guess.

At load it runs check.npz -- pictures the trainer scored -- and refuses the
model if the scores don't match: a model copied wrong, or preprocessed
differently here than in training, says so instead of misreading the board.
"""
import json
import os

import numpy as np

import config

MIN_CONFIDENCE = 0.8


def latest_model(models_dir=None):
    models_dir = models_dir or config.MODELS_DIR
    if not os.path.isdir(models_dir):
        return None
    found = sorted(d for d in os.listdir(models_dir)
                   if d[:1] == "v" and d[1:].isdigit()
                   and os.path.exists(os.path.join(models_dir, d, "model.onnx")))
    return os.path.join(models_dir, found[-1]) if found else None


def softmax(scores):
    e = np.exp(scores - scores.max(axis=1, keepdims=True))
    return e / e.sum(axis=1, keepdims=True)


class Classifier(object):
    def __init__(self, model_dir=None):
        import cv2
        from vision.square_cnn import to_input
        self.dir = model_dir or latest_model()
        if self.dir is None:
            raise RuntimeError("no trained model in %s yet (train one: python3 -m learn.train)"
                               % config.MODELS_DIR)
        with open(os.path.join(self.dir, "manifest.json")) as fh:
            self.manifest = json.load(fh)
        self.labels = self.manifest["labels"]
        self.pre = self.manifest["preprocess"]
        self._to_input = to_input
        self.net = cv2.dnn.readNetFromONNX(os.path.join(self.dir, "model.onnx"))
        self.check()

    def scores(self, images):
        self.net.setInput(self._to_input(images, self.pre))
        return np.array(self.net.forward())

    def check(self, tolerance=1e-2):
        path = os.path.join(self.dir, "check.npz")
        if not os.path.exists(path):
            raise RuntimeError("%s has no check.npz -- can't trust it" % self.dir)
        data = np.load(path)
        got = self.scores(list(data["images"]))
        want = data["scores"]
        worst = float(np.abs(got - want).max())
        if worst > tolerance * max(1.0, float(np.abs(want).max())):
            raise RuntimeError("%s gives different answers here than when it was trained "
                               "(off by %.3f) -- copy it again" % (self.dir, worst))
        return worst

    def classify(self, crops):
        """{square: (label, confidence)} for {square: image}."""
        names = list(crops)
        probs = softmax(self.scores([crops[sq] for sq in names]))
        return {sq: (self.labels[int(p.argmax())], float(p.max())) for sq, p in zip(names, probs)}

    def unsure(self, result):
        return sorted(sq for sq, (_, conf) in result.items() if conf < MIN_CONFIDENCE)
