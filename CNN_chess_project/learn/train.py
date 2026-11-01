#!/usr/bin/env python3
"""
Trains the square CNN (vision/square_cnn.py) -- on the NVIDIA GPU when
there is one (CUDA), otherwise on the CPU (slower, still works).

    python3 -m learn.make_squares      # photos -> labelled square pictures
    python3 -m learn.train             # -> models/v001/ (then v002, ...)

Output: models/vNNN/ with everything needed to trust or rebuild the model:
    model.onnx      the network, for the Jetson (OpenCV dnn)
    model.pt        the same weights, for PyTorch
    manifest.json   labels in order, input size, colour order, preprocessing,
                    when, from which sessions, and the test scores
    check.npz       a few test inputs + the scores this model gives them:
                    vision/classifier.py runs them at load and refuses a model
                    that doesn't give the same answers (Microduck's lesson:
                    validate at load, so the PC and the Jetson can't disagree)
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config                                           # noqa: E402
from learn import dataset, evaluate                     # noqa: E402
from vision import square_cnn                           # noqa: E402


def next_model_dir(models_dir):
    if not os.path.isdir(models_dir):
        os.makedirs(models_dir)
    taken = [int(d[1:]) for d in os.listdir(models_dir) if d[:1] == "v" and d[1:].isdigit()]
    path = os.path.join(models_dir, "v%03d" % (max(taken or [0]) + 1))
    os.makedirs(path)
    return path


def export_onnx(torch, model, path):
    model.eval()
    dummy = torch.zeros(1, 3, square_cnn.INPUT_H, square_cnn.INPUT_W)
    kwargs = dict(input_names=["input"], output_names=["scores"], opset_version=11,
                  dynamic_axes={"input": {0: "batch"}, "scores": {0: "batch"}})
    try:
        torch.onnx.export(model, dummy, path, dynamo=False, **kwargs)
    except TypeError:                                 # older PyTorch: no dynamo option
        torch.onnx.export(model, dummy, path, **kwargs)


def train(squares_dir=None, models_dir=None, epochs=15, batch=128, lr=2e-3, seed=0,
          log=print):
    import numpy as np
    import torch
    from torch.utils.data import DataLoader

    squares_dir = squares_dir or os.path.join(config.DATA_DIR, "squares")
    models_dir = models_dir or config.MODELS_DIR
    labels = list(square_cnn.LABELS)
    samples = dataset.scan(squares_dir, labels)
    if not samples:
        log("No square pictures in %s -- run  python3 -m learn.make_squares  first." % squares_dir)
        return None
    train_s, test_s = dataset.split(samples)
    torch.manual_seed(seed)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    log("Training on %s%s: %d square pictures (%d test, from %s)" % (
        device.upper(), " (%s)" % torch.cuda.get_device_name(0) if device == "cuda" else "",
        len(train_s), len(test_s), ", ".join(sorted({s[2] for s in test_s})) or "-"))
    counts = [sum(1 for s in train_s if s[1] == k) for k in range(len(labels))]
    log("  " + ", ".join("%s %d" % (lab, c) for lab, c in zip(labels, counts)))

    pre = square_cnn.PREPROCESS
    loader = DataLoader(dataset.SquareDataset(train_s, pre, augment=True, seed=seed),
                        batch_size=batch, shuffle=True, num_workers=0)
    model = square_cnn.build(len(labels)).to(device)
    # rare labels count for more (a board is mostly empty squares)
    total = float(sum(counts))
    weights = torch.tensor([total / (len(labels) * max(c, 1)) for c in counts],
                           dtype=torch.float32, device=device)
    loss_fn = torch.nn.CrossEntropyLoss(weight=weights)
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=max(1, epochs))
    t0 = time.time()
    for epoch in range(1, epochs + 1):
        model.train()
        seen, right, loss_sum = 0, 0, 0.0
        for x, y in loader:
            x = torch.as_tensor(np.asarray(x)).to(device)
            y = torch.as_tensor(np.asarray(y)).long().to(device)
            opt.zero_grad()
            out = model(x)
            loss = loss_fn(out, y)
            loss.backward()
            opt.step()
            seen += len(y)
            right += int((out.argmax(1) == y).sum())
            loss_sum += float(loss.detach()) * len(y)
        sched.step()
        log("  epoch %2d/%d  loss %.3f  train accuracy %.1f%%" % (
            epoch, epochs, loss_sum / max(seen, 1), 100.0 * right / max(seen, 1)))

    model.eval()
    model_cpu = model.to("cpu")

    def predict(images):
        with torch.no_grad():
            return model_cpu(torch.as_tensor(square_cnn.to_input(images, pre))).numpy()

    metrics = evaluate.score(predict, test_s, labels)
    log(evaluate.describe(metrics, labels))

    out = next_model_dir(models_dir)
    torch.save(model_cpu.state_dict(), os.path.join(out, "model.pt"))
    export_onnx(torch, model_cpu, os.path.join(out, "model.onnx"))
    # the check: a few test (or training) pictures and this model's scores for them
    import cv2
    check_s = (test_s or train_s)[:16]
    imgs = [cv2.imread(s[0]) for s in check_s]
    np.savez_compressed(os.path.join(out, "check.npz"),
                        images=np.stack(imgs).astype(np.uint8), scores=predict(imgs))
    manifest = {"labels": labels, "preprocess": pre, "input": [3, pre["height"], pre["width"]],
                "made": time.strftime("%Y-%m-%d %H:%M"), "device": device,
                "epochs": epochs, "train_seconds": round(time.time() - t0, 1),
                "train_sessions": sorted({s[2] for s in train_s}),
                "test_sessions": sorted({s[2] for s in test_s}),
                "train_squares": len(train_s), "test_squares": len(test_s),
                "test": metrics, "code_version": config.VERSION}
    with open(os.path.join(out, "manifest.json"), "w") as fh:
        json.dump(manifest, fh, indent=2)
    log("Saved %s  (copy the folder to the Jetson's models/ to use it)" % out)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Train the square CNN")
    ap.add_argument("--epochs", type=int, default=15)
    ap.add_argument("--batch", type=int, default=128)
    ap.add_argument("--squares", default=None, help="folder of square pictures")
    args = ap.parse_args()
    train(args.squares, epochs=args.epochs, batch=args.batch)
