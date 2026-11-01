#!/usr/bin/env python3
"""
Grades a model on the held-out test photos. Two scores, because one isn't
enough:
  * confusion matrix -- which labels get mixed up (e.g. black on dark squares)
  * whole-board accuracy -- the share of test photos where ALL 64 squares
    are right. One wrong square = a wrong move, so this is the number that
    matters.
"""


def score(predict, samples, labels, batch=256):
    """predict(list of images) -> scores array. samples from learn/dataset.py."""
    import cv2
    n = len(labels)
    confusion = [[0] * n for _ in range(n)]
    photo_ok = {}
    for i in range(0, len(samples), batch):
        part = samples[i:i + batch]
        scores = predict([cv2.imread(s[0]) for s in part])
        for s, row in zip(part, scores):
            guess = int(max(range(n), key=lambda k: row[k]))
            confusion[s[1]][guess] += 1
            photo_ok[s[3]] = photo_ok.get(s[3], True) and guess == s[1]
    total = sum(sum(r) for r in confusion)
    right = sum(confusion[k][k] for k in range(n))
    return {"squares": total,
            "square_accuracy": round(right / float(total), 4) if total else None,
            "photos": len(photo_ok),
            "board_accuracy": (round(sum(photo_ok.values()) / float(len(photo_ok)), 4)
                               if photo_ok else None),
            "confusion": confusion}


def describe(m, labels):
    if not m["squares"]:
        return "  (no test pictures: collect a second session to get a fair test score)"
    lines = ["  Test: %.1f%% of %d squares right; %.0f%% of %d photos had ALL 64 right"
             % (100 * m["square_accuracy"], m["squares"], 100 * m["board_accuracy"], m["photos"]),
             "  confusion (rows = truth, columns = guess): " + "  ".join(labels)]
    for lab, row in zip(labels, m["confusion"]):
        lines.append("    %-6s %s" % (lab, "  ".join("%5d" % v for v in row)))
    return "\n".join(lines)
