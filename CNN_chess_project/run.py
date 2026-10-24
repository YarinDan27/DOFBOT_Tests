#!/usr/bin/env python3
"""
DOFBOT chess -- start here:

    cd ~/YDscripts/CNN_chess_project && python3 run.py

On start it says where it thinks the claw is and waits for your Enter before
moving. Then it goes home -- the look-at-board pose you taught (option 2), or
the ready pose until you've taught one -- slowly, in small steps, each checked
against what the servos report back. If a joint falls short (something in the
way, or the arm wasn't where the program thought), it stops and holds still.

Ctrl-C never moves the arm: it stops right where it is. Option 8 parks and quits.

This menu (and the sessions/ it starts) is the only code that connects the
arm, the camera and the game. robot/, vision/ and game/ never import each
other, so each can be tested on its own.
"""
import config
from robot.arm import Arm, ArmBlocked, OutOfReach, UnsafeMove
from robot.board_frame import BoardFrame, format_grid, is_square
from sessions import look_pose, self_test, teach


def start_camera(state):
    """Starts the camera the first time it's needed. Carries on without it if it fails."""
    if state["camera"] is None:
        try:
            from vision.camera import Camera
            camera = Camera()
            camera.start(stream=True)
            state["camera"] = camera
        except Exception as e:
            print("  Camera unavailable: %s" % e)
            print("  Carrying on without photos.")
    return state["camera"]


def back_home(arm):
    """Rise straight up first, THEN swing home: never swing low."""
    arm.lift_clear()
    arm.go_home()


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
        print("Board: not taught yet -- option 1.")
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
    arm.goto(x, y, z + config.HOVER_MM)
    input("  Hovering over %s. Is it centred? Press Enter to go back. " % sq)
    back_home(arm)


def move_piece_test(arm, frame):
    src = ask_square("Move the piece on: ", frame)
    if src is None:
        return
    dst = ask_square("To: ", frame)
    if dst is None:
        return
    input("  Moving %s -> %s. Keep hands clear and press Enter. " % (src, dst))
    gripped = arm.move_piece(frame.square_xyz(src), frame.square_xyz(dst))
    print("  " + ("Gripped something." if gripped else "Claw closed on nothing."))
    back_home(arm)


def report_block(e):
    print("\n  STOPPED: %s." % e)
    print("  The arm is holding still where it stopped. Look at it before doing "
          "anything else.")
    print("  (Every move is in logs/motion/ -- send me the newest file.)")


def run_safely(arm, action, *args):
    """Runs a menu action; a blocked move stops it cleanly instead of crashing."""
    if arm.blocked:
        print("\nThe last move was blocked, so the arm is holding still where it stopped.")
        ans = input("Look at it. Is the way clear now? y = yes, carry on   "
                    "anything else = no: ")
        if ans.strip().lower() != "y":
            return None
        arm.clear_block()
    try:
        return action(*args)
    except ArmBlocked as e:
        report_block(e)
    except (OutOfReach, UnsafeMove) as e:
        print("  " + str(e))
    except RuntimeError as e:
        print("  Stopped: %s." % e)
    return None


def main():
    print("\n=== DOFBOT Chess (CNN) v%s ===" % config.VERSION)
    state = {"frame": BoardFrame.load(), "camera": None}
    show_board(state["frame"])
    home = look_pose.load()
    print("Home: " + ("the look-at-board pose" if home else
                      "the ready pose (teach the look-at-board pose with option 2)"))

    with Arm() as arm:
        arm.set_board(state["frame"])
        if home:
            arm.home_pose = home
        try:
            arm.start_up()
        except ArmBlocked as e:
            report_block(e)
        except RuntimeError as e:
            print("Couldn't start: %s." % e)
            return

        while True:
            print("\n1. Teach the board by hand (servos off)")
            print("2. Teach the look-at-board pose (servos off)")
            print("3. Self-test: how well can it reach each square?")
            print("4. Reach map")
            print("5. Hover over one square")
            print("6. Move one piece")
            print("7. Camera on/off (shows which camera and its frame rate)")
            print("8. Park and quit")
            try:
                choice = input("Select: ").strip()
            except (KeyboardInterrupt, EOFError):
                print()
                arm.skip_park = True
                break

            frame = state["frame"]
            if choice == "1":
                new = run_safely(arm, teach.run, arm, start_camera(state))
                if new is not None:
                    state["frame"] = new
                    arm.set_board(new)
            elif choice == "2":
                run_safely(arm, look_pose.run, arm, start_camera(state))
            elif choice == "3":
                run_safely(arm, self_test.run, arm, frame, start_camera(state))
            elif choice == "4":
                if frame is None:
                    print("Teach the board first.")
                else:
                    show_board(frame)
                    print("ok = can grab AND hover, low = grab only, -- = out of reach")
                    print(format_grid(frame.reach_map()))
            elif choice in ("5", "6"):
                if frame is None:
                    print("Teach the board first.")
                elif choice == "5":
                    run_safely(arm, hover_test, arm, frame)
                else:
                    run_safely(arm, move_piece_test, arm, frame)
            elif choice == "7":
                if state["camera"] is None:
                    camera = start_camera(state)
                    if camera is not None:
                        print("  " + camera.stats())
                else:
                    state["camera"].stop()
                    state["camera"] = None
                    print("  Camera off.")
            elif choice == "8":
                break

    if state["camera"] is not None:
        state["camera"].stop()
    print("Bye!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped. (Ctrl-C never moves the arm.)")
