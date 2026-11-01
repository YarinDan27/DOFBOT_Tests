#!/usr/bin/env python3
"""
Feeds the square pictures to PyTorch.

Splits by SESSION, never by photo: photos from one session look almost the
same (same light, same pieces in the same places), so letting one session
into both training and testing would make the test score lie. The newest
session is the test set (or, with only one session, its last 20% of photos).
"""
import os
import random


def scan(squares_dir, labels):
    """[(path, label index, session, photo id)] for every square picture."""
    out = []
    if not os.path.isdir(squares_dir):
        return out
    for session in sorted(os.listdir(squares_dir)):
        sdir = os.path.join(squares_dir, session)
        if not os.path.isdir(sdir):
            continue
        for k, label in enumerate(labels):
            ldir = os.path.join(sdir, label)
            if not os.path.isdir(ldir):
                continue
            for name in sorted(os.listdir(ldir)):
                if name.endswith(".png"):
                    out.append((os.path.join(ldir, name), k, session,
                                session + "/" + name.split("_")[0]))
    return out


def split(samples, test_share=0.2):
    """(train, test): the newest session is the test set."""
    sessions = sorted({s[2] for s in samples})
    if len(sessions) >= 2:
        test_sessions = {sessions[-1]}
        return ([s for s in samples if s[2] not in test_sessions],
                [s for s in samples if s[2] in test_sessions])
    photos = sorted({s[3] for s in samples})
    cut = max(1, int(round(len(photos) * (1 - test_share))))
    test_photos = set(photos[cut:])
    return ([s for s in samples if s[3] not in test_photos],
            [s for s in samples if s[3] in test_photos])


class SquareDataset(object):
    """A torch Dataset (duck-typed: torch is only imported by the trainer)."""

    def __init__(self, samples, pre, augment=False, seed=0):
        self.samples = samples
        self.pre = pre
        self.augment = augment
        self.rng = random.Random(seed)

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, i):
        import cv2
        import numpy as np
        from vision.square_cnn import to_input
        path, label, _, _ = self.samples[i]
        img = cv2.imread(path)
        if self.augment:
            img = self._jitter(img, np, cv2)
        return to_input([img], self.pre)[0], label

    def _jitter(self, img, np, cv2):
        """Light, contrast, a few pixels of shift and a slight turn: the
        camera and the room are never quite the same twice."""
        rng = self.rng
        h, w = img.shape[:2]
        a = rng.uniform(0.7, 1.3)                       # contrast
        b = rng.uniform(-30, 30)                        # brightness
        img = np.clip(img.astype(np.float32) * a + b, 0, 255).astype(np.uint8)
        m = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), rng.uniform(-4, 4), rng.uniform(0.95, 1.05))
        m[0, 2] += rng.uniform(-3, 3)
        m[1, 2] += rng.uniform(-3, 3)
        img = cv2.warpAffine(img, m, (w, h), borderMode=cv2.BORDER_REPLICATE)
        if rng.random() < 0.5:
            img = img[:, ::-1].copy()                   # mirror: a piece is still that piece
        return img
