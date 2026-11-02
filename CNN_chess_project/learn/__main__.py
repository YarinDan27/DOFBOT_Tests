#!/usr/bin/env python3
"""
The whole camera training in one command, on the PC with the NVIDIA GPU:

    python3 -m learn                       # uses config.JETSON_SSH
    python3 -m learn yarin@10.0.0.173      # or say the Jetson's login

  1. copies the arm's photos here    (Jetson data/raw -> data/raw)
  2. cuts them into square pictures  (learn/make_squares.py)
  3. trains the CNN on the GPU        (learn/train.py -> models/vNNN)
  4. copies the new model back to the Jetson's models/

The photos come from the arm's own camera (menu 9). No phone needed.
Tip: run  ssh-copy-id <login>  once, and it stops asking for the password.
"""
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config                                   # noqa: E402
from learn import make_squares, train           # noqa: E402


def run(cmd):
    print("  $ " + " ".join(cmd))
    return subprocess.call(cmd) == 0


def main(argv):
    host = argv[0] if argv else config.JETSON_SSH
    remote = config.JETSON_DIR
    print("1/4  Copying the arm's photos from %s ..." % host)
    if not os.path.isdir(config.DATA_DIR):
        os.makedirs(config.DATA_DIR)
    if not run(["scp", "-r", "%s:%s/data/raw" % (host, remote), config.DATA_DIR]):
        print("Couldn't copy the photos. Is the Jetson on, and is %s its login? "
              "(python3 -m learn <login>@<address>)" % host)
        return 1
    print("2/4  Cutting them into square pictures ...")
    make_squares.main()
    print("3/4  Training ...")
    out = train.train()
    if out is None:
        return 1
    print("4/4  Sending %s back to the arm ..." % os.path.basename(out))
    if not (run(["ssh", host, "mkdir -p %s/models" % remote])
            and run(["scp", "-r", out, "%s:%s/models/" % (host, remote)])):
        print("Couldn't copy it back. Copy the folder %s to the Jetson's models/ yourself." % out)
        return 1
    print("Done. On the arm: menu 9, then r reads the board.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
