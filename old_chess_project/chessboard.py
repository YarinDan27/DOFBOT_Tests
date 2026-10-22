#!/usr/bin/env python3
"""
Where the board is, learned by touching three squares. Nothing hardcoded.

Any three squares work as long as they're not in a straight line -- useful
when a corner is out of reach. The math solves:

    position(file, rank) = origin + file * file_step + rank * rank_step

for origin, file_step and rank_step, given three known squares.
"""
import json
import os
import math

FILES = "abcdefgh"
CALIBRATION_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "board_calibration.json")


def square_to_indexes(square):
    """'e4' -> (4, 3): file index a=0..h=7, rank index 1=0..8=7"""
    square = square.strip().lower()
    if len(square) != 2 or square[0] not in FILES or square[1] not in "12345678":
        raise ValueError(f"bad square: {square}")
    return FILES.index(square[0]), int(square[1]) - 1


def _solve_3x3(matrix, rhs):
    """Cramer's rule. matrix is 3x3, rhs is length 3."""
    def det3(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))

    base = det3(matrix)
    if abs(base) < 1e-9:
        raise ValueError("those three squares are in a straight line -- pick others")
    out = []
    for col in range(3):
        swapped = [row[:] for row in matrix]
        for row in range(3):
            swapped[row][col] = rhs[row]
        out.append(det3(swapped) / base)
    return out


class BoardFrame:
    def __init__(self, origin, file_step, rank_step):
        self.origin = list(origin)
        self.file_step = list(file_step)
        self.rank_step = list(rank_step)

    @classmethod
    def from_points(cls, points):
        """points: {'a1': (x,y,z), 'h1': (...), 'a6': (...)} -- any 3 squares."""
        if len(points) != 3:
            raise ValueError("need exactly three squares")
        squares = list(points.keys())
        matrix = []
        for square in squares:
            f, r = square_to_indexes(square)
            matrix.append([1.0, float(f), float(r)])

        origin, file_step, rank_step = [], [], []
        for axis in range(3):
            rhs = [float(points[sq][axis]) for sq in squares]
            o, u, v = _solve_3x3([row[:] for row in matrix], rhs)
            origin.append(o)
            file_step.append(u)
            rank_step.append(v)
        return cls(origin, file_step, rank_step)

    def square_xyz(self, square):
        f, r = square_to_indexes(square)
        return tuple(self.origin[i] + self.file_step[i] * f + self.rank_step[i] * r
                     for i in range(3))

    def square_size_mm(self):
        file_mm = math.sqrt(sum(v * v for v in self.file_step))
        rank_mm = math.sqrt(sum(v * v for v in self.rank_step))
        return file_mm, rank_mm

    def to_dict(self):
        return {"origin": self.origin,
                "file_step": self.file_step,
                "rank_step": self.rank_step}

    def save(self, path=CALIBRATION_PATH):
        with open(path, "w") as fh:
            json.dump(self.to_dict(), fh, indent=2)

    @classmethod
    def load(cls, path=CALIBRATION_PATH):
        if not os.path.exists(path):
            return None
        with open(path) as fh:
            d = json.load(fh)
        if "origin" in d:
            return cls(d["origin"], d["file_step"], d["rank_step"])
        # older file that stored three corner points
        return cls.from_points({k: tuple(v) for k, v in d.items()})
