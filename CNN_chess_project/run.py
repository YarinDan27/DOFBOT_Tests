#!/usr/bin/env python3
"""
DOFBOT chess -- start here:

    cd ~/YDscripts/CNN_chess_project && python3 run.py

This menu (and the sessions/ it starts) is the only code that connects the
arm, the camera and the game. robot/, vision/ and game/ never import each
other, so each can be tested on its own.
"""
import config
from robot.arm import Arm, OutOfReach
from robot.board_frame import BoardFrame, format_grid, is_square
from sessions import self_test, teach


def start_camera(state):
    """Starts the camera the first time it's needed. Carries on without it if it fails."""
    if state["camera"] is None:
        try:
            from vision.camera import Camera
            camera = Camera()
            camera.start(stream=True)
            state["camera"] = camera
            print("  Camera on. Watch live: " + camera.url())
        except Exception as e:
            print("  Camera unavailable (%s). Carrying on without photos." % e)
    return state["camera"]


def back_to_ready(arm):
    """Rise straight up first, THEN swing to the ready pose: never swing low."""
    arm.lift_clear()
    arm.ready()


def ask_square(prompt, frame):
    raw = input(prompt).strip().lower()
    if not is_square(raw):
        print("  Not a square.")
        return None
    if frame.reach_map()[raw] != "ok":
        print("  %s is out of reach." % raw)
        return None
    return raw


def show_board(frame):
    if frame is None:
        print("Board: not taught yet -- start with 1.")
        return
    file_mm, rank_mm = frame.square_size_mm()
    residuals = frame.residuals()
    worst = max(residuals.values()) if residuals else 0.0
    reach = frame.reach_map()
    out = sum(1 for v in reach.values() if v != "ok")
    print("Board: %d squares taught, squares %.1f x %.1f mm, worst touch %.1f mm, "
          "%d of 64 out of reach" % (len(frame.taught), file_mm, rank_mm, worst, out))


def hover_test(arm, frame):
    sq = ask_square("Square to hover over: ", frame)
    if sq is None:
        return
    x, y, z = frame.square_xyz(sq)
    try:
        arm.goto(x, y, z + config.HOVER_MM)
    except OutOfReach as e:
        print("  " + str(e))
        return
    input("  Hovering over %s. Is it centred? Press Enter to go back. " % sq)
    back_to_ready(arm)


def move_piece_test(arm, frame):
    src = ask_square("Move the piece on: ", frame)
    if src is None:
        return
    dst = ask_square("To: ", frame)
    if dst is None:
        return
    input("  Moving %s -> %s. Keep hands clear and press Enter. " % (src, dst))
    try:
        gripped = arm.move_piece(frame.square_xyz(src), frame.square_xyz(dst))
        print("  " + ("Gripped something." if gripped else "Claw closed on nothing."))
    except OutOfReach as e:
        print("  " + str(e))
    back_to_ready(arm)


def main():
    print("\n=== DOFBOT Chess (CNN) v%s ===" % config.VERSION)
    state = {"frame": BoardFrame.load(), "camera": None}
    show_board(state["frame"])

    with Arm() as arm:
        arm.ready()
        while True:
            print("\n1. Teach the board by hand (servos off)")
            print("2. Self-test: how well can it reach each square?")
            print("3. Reach map")
            print("4. Hover over one square")
            print("5. Move one piece")
            print("6. Camera stream on/off")
            print("7. Quit")
            try:
                choice = input("Select: ").strip()
            except (KeyboardInterrupt, EOFError):
                print()
                break

            frame = state["frame"]
            if choice == "1":
                new = teach.run(arm, start_camera(state))
                if new is not None:
                    state["frame"] = new
            elif choice == "2":
                self_test.run(arm, frame, start_camera(state))
            elif choice == "3":
                if frame is None:
                    print("Teach the board first.")
                else:
                    show_board(frame)
                    print("ok = can grab AND hover, low = grab only, -- = out of reach")
                    print(format_grid(frame.reach_map()))
            elif choice in ("4", "5"):
                if frame is None:
                    print("Teach the board first.")
                elif choice == "4":
                    hover_test(arm, frame)
                else:
                    move_piece_test(arm, frame)
            elif choice == "6":
                if state["camera"] is None:
                    start_camera(state)
                else:
                    state["camera"].stop()
                    state["camera"] = None
                    print("  Camera off.")
            elif choice == "7":
                break

    if state["camera"] is not None:
        state["camera"].stop()
    print("Parked. Bye!")


if __name__ == "__main__":
    main()
