#!/usr/bin/env python3
"""
Where the board is, in the arm's millimetres.

The model:

    position(file, rank) = origin + file * file_step + rank * rank_step

(file a=0..h=7, rank 1=0..8=7).

Seen from above, a real board is a rigid grid of SQUARE squares at right
angles, so the fit is only allowed to slide, rotate and scale it (plus tilt
it, for height). That's 7 numbers to find, however many squares you teach:

    x = ox + a*file - b*rank        (a, b) = file step across the table
    y = oy + b*file + a*rank        (-b, a) = rank step: same length, 90 deg turned
    z = oz + c*file + d*rank        a plane: the board may be slightly tilted

All of those are linear in the unknowns, so plain least squares solves them.

Teach 4+ squares and every square gets a "residual": how far your touch is
from the board that best agrees with all the others. Because the grid can't
stretch or skew to hide a mistake, a bad touch shows up as a big residual.

Grew out of old_chess_project/chessboard.py, whose 3-point solve could bend
the grid to fit any three touches -- including wrong ones.
"""
import json
import math
import os

import config
from robot import kinematics

FILES = "abcdefgh"


def square_to_indexes(square):
    """'e4' -> (4, 3): file index a=0..h=7, rank index 1=0..8=7"""
    square = square.strip().lower()
    if len(square) != 2 or square[0] not in FILES or square[1] not in "12345678":
        raise ValueError("bad square: %r" % square)
    return FILES.index(square[0]), int(square[1]) - 1


def is_square(text):
    try:
        square_to_indexes(text)
        return True
    except ValueError:
        return False


def all_squares():
    """a1, b1 ... h8"""
    return [f + str(r) for r in range(1, 9) for f in FILES]


def least_squares(rows, values):
    """
    Finds p that best satisfies  row . p = value  for every (row, value),
    by solving the normal equations (A^T A) p = A^T b with Gaussian
    elimination. Small and dependency-free, so it runs anywhere.
    """
    n = len(rows[0])
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(n)] for i in range(n)]
    atb = [sum(r[i] * v for r, v in zip(rows, values)) for i in range(n)]
    m = [ata[i] + [atb[i]] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda k: abs(m[k][col]))
        if abs(m[pivot][col]) < 1e-9:
            raise ValueError("those squares are all in a straight line -- "
                             "teach at least one off that line")
        m[col], m[pivot] = m[pivot], m[col]
        for k in range(n):
            if k != col:
                factor = m[k][col] / m[col][col]
                m[k] = [m[k][j] - factor * m[col][j] for j in range(n + 1)]
    return [m[i][n] / m[i][i] for i in range(n)]


def _length(v):
    return math.sqrt(sum(c * c for c in v))


class BoardFrame:
    def __init__(self, origin, file_step, rank_step, taught=None):
        self.origin = list(origin)
        self.file_step = list(file_step)
        self.rank_step = list(rank_step)
        self.taught = dict(taught or {})   # square -> [x, y, z] as touched

    # ---------------- building one ----------------
    @classmethod
    def fit(cls, points):
        """
        points: {'b2': (x, y, z), ...} -- 3 or more squares, not all in a line.
        """
        if len(points) < 3:
            raise ValueError("need at least three squares")

        # across the table: unknowns ox, oy, a, b
        rows, values = [], []
        for square, (x, y, _) in points.items():
            f, r = square_to_indexes(square)
            rows.append([1.0, 0.0, float(f), -float(r)])
            values.append(float(x))
            rows.append([0.0, 1.0, float(r), float(f)])
            values.append(float(y))
        # height: unknowns oz, c, d
        z_rows, z_values = [], []
        for square, (_, _, z) in points.items():
            f, r = square_to_indexes(square)
            z_rows.append([1.0, float(f), float(r)])
            z_values.append(float(z))

        oz, c, d = least_squares(z_rows, z_values)   # raises if all in a line
        ox, oy, a, b = least_squares(rows, values)
        taught = {sq: [float(v) for v in xyz] for sq, xyz in points.items()}
        return cls([ox, oy, oz], [a, b, c], [-b, a, d], taught)

    # ---------------- using one ----------------
    def square_xyz(self, square):
        f, r = square_to_indexes(square)
        return tuple(self.origin[i] + self.file_step[i] * f + self.rank_step[i] * r
                     for i in range(3))

    def residuals(self):
        """square -> mm between where you touched and where the fitted board says."""
        out = {}
        for square, xyz in self.taught.items():
            fitted = self.square_xyz(square)
            out[square] = _length([fitted[i] - xyz[i] for i in range(3)])
        return out

    def square_size_mm(self):
        return _length(self.file_step), _length(self.rank_step)

    def corner_angle_deg(self):
        """Angle between the files and the ranks. A real board says ~90."""
        u, v = self.file_step, self.rank_step
        cos_a = sum(u[i] * v[i] for i in range(3)) / (_length(u) * _length(v))
        return math.degrees(math.acos(max(-1.0, min(1.0, cos_a))))

    def reach_map(self, zone=None):
        """
        square -> 'ok'  : reachable at grabbing height AND hover height
                  'low' : grabbing height only (arm can't hover above it)
                  '--'  : out of reach
        Both heights matter: every pick-up starts from the hover point.
        With a World (robot/world.py): only counting poses whose shape is
        clear of the electronics, base and table.
        """
        def clear(sol):
            return zone.margin([sol[1], sol[2], sol[3], sol[4]])[0] >= 0.0

        accept = clear if zone is not None and hasattr(zone, "margin") else None
        out = {}
        for square in all_squares():
            x, y, z = self.square_xyz(square)
            grab = kinematics.solve_reachable(x, y, z, accept)[0] is not None
            hover = kinematics.solve_reachable(x, y, z + config.HOVER_MM, accept)[0] is not None
            out[square] = "ok" if (grab and hover) else ("low" if grab else "--")
        return out

    # ---------------- saving ----------------
    def to_dict(self):
        file_mm, rank_mm = self.square_size_mm()
        return {"origin": self.origin,
                "file_step": self.file_step,
                "rank_step": self.rank_step,
                "taught": self.taught,
                "residuals_mm": {k: round(v, 2) for k, v in self.residuals().items()},
                "square_mm": [round(file_mm, 2), round(rank_mm, 2)],
                "corner_angle_deg": round(self.corner_angle_deg(), 2)}

    def save(self, path=config.BOARD_FILE):
        folder = os.path.dirname(path)
        if folder and not os.path.isdir(folder):
            os.makedirs(folder)
        with open(path, "w") as fh:
            json.dump(self.to_dict(), fh, indent=2)

    @classmethod
    def load(cls, path=config.BOARD_FILE):
        if not os.path.exists(path):
            return None
        with open(path) as fh:
            d = json.load(fh)
        return cls(d["origin"], d["file_step"], d["rank_step"], d.get("taught"))


def format_grid(cells, width=5):
    """
    cells: square -> short text. Prints the board the way you see it from
    behind the arm (rank 8 at the top).
    """
    lines = []
    for r in range(8, 0, -1):
        row = "".join(str(cells.get(f + str(r), "")).rjust(width) for f in FILES)
        lines.append("%d %s" % (r, row))
    lines.append("  " + "".join(f.rjust(width) for f in FILES))
    return "\n".join(lines)
