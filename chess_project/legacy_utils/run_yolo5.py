#!/usr/bin/env python3

import argparse, os, pathlib, subprocess, sys

home = pathlib.Path.home()
YOLO_DIR = home / "yolov5"                 # real repo (v3.1)
DETECT = YOLO_DIR / "detect.py"
WEIGHTS = YOLO_DIR / "yolov5s_v3.1.pt"

def main():
    arm.Arm_serial_servo_write6(90, 0, 180, 180, 90,100 , 100, 500)
    if not DETECT.exists():
        sys.exit(f"[!] detect.py not found at {DETECT}. Expected repo at {YOLO_DIR} (tag v3.1).")

    os.environ.setdefault("MPLCONFIGDIR", str(home / ".config" / "matplotlib"))
    os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)

    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=str(WEIGHTS))
    ap.add_argument("--source", default="0")
    ap.add_argument("--device", default="0")   # '0' or 'cpu'
    ap.add_argument("--img", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--project", default="runs/detect")
    ap.add_argument("--name", default="exp")
    ap.add_argument("--save-txt", action="store_true")
    args = ap.parse_args()

    cmd = [
        sys.executable, str(DETECT),
        "--weights", args.weights,
        "--source", args.source,
        "--device", args.device,
        "--img-size", str(args.img),
        "--conf-thres", str(args.conf),
        "--project", args.project,
        "--name", args.name,
    ]
    if args.save_txt:
        cmd.append("--save-txt")

    print("[*] Running:", " ".join(cmd))
    subprocess.run(cmd, cwd=str(YOLO_DIR), check=True)

if __name__ == "__main__":
    from Arm_Lib import Arm_Device
    arm = Arm_Device
    main()

