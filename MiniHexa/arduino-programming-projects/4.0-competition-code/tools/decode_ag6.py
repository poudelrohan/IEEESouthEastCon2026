#!/usr/bin/env python3
"""
decode_ag6.py — Decode AG6 (Obstacle Crossing) .rob file

Reads the JSON .rob file and prints per-frame analysis:
- Which legs are lifting (B-servo deviation from home)
- The tripod grouping pattern
- Servo direction conventions

Servo mapping (18 servos, 3 per leg):
  Servo  1,2,3  → Leg 1 (rear-right)   joints A, B, C
  Servo  4,5,6  → Leg 2 (mid-right)    joints A, B, C
  Servo  7,8,9  → Leg 3 (front-right)  joints A, B, C
  Servo 10,11,12 → Leg 4 (front-left)  joints A, B, C
  Servo 13,14,15 → Leg 5 (mid-left)    joints A, B, C
  Servo 16,17,18 → Leg 6 (rear-left)   joints A, B, C

280-degree servos: midpoint = 1500, factor = 7.143 duty/degree
B-servo lift direction:
  Right legs (1,2,3): B < 1500 = lift UP
  Left legs  (4,5,6): B > 1500 = lift UP
"""

import json
import sys
import os

# Servo-to-leg mapping
LEGS = {
    1: {"name": "Leg1 (rear-R)",  "servos": (0, 1, 2),  "side": "right"},
    2: {"name": "Leg2 (mid-R)",   "servos": (3, 4, 5),  "side": "right"},
    3: {"name": "Leg3 (front-R)", "servos": (6, 7, 8),  "side": "right"},
    4: {"name": "Leg4 (front-L)", "servos": (9, 10, 11), "side": "left"},
    5: {"name": "Leg5 (mid-L)",   "servos": (12, 13, 14), "side": "left"},
    6: {"name": "Leg6 (rear-L)",  "servos": (15, 16, 17), "side": "left"},
}

# Home position B-servo values (from frame 1)
HOME_B_RIGHT = 1409  # right-side legs home B value
HOME_B_LEFT  = 1590  # left-side legs home B value

DUTY_PER_DEGREE = 7.143


def load_rob(path):
    with open(path, "r") as f:
        data = json.load(f)
    return data["Actions"]


def analyze_frame(frame_idx, servos, duration):
    print(f"\n{'='*60}")
    print(f"Frame {frame_idx+1:2d}  |  Duration: {duration} ms")
    print(f"{'='*60}")

    lifting = []
    stepping = []

    for leg_id, info in LEGS.items():
        a_idx, b_idx, c_idx = info["servos"]
        a_val = servos[a_idx]
        b_val = servos[b_idx]
        c_val = servos[c_idx]

        home_b = HOME_B_RIGHT if info["side"] == "right" else HOME_B_LEFT

        b_dev = b_val - home_b
        a_dev = a_val - 1500
        c_home = 1356 if info["side"] == "right" else 1642
        c_dev = c_val - c_home

        # Determine if leg is lifting
        if info["side"] == "right":
            is_lifting = b_dev < -50  # B goes below home = lift for right
            lift_amount = -b_dev
        else:
            is_lifting = b_dev > 50   # B goes above home = lift for left
            lift_amount = b_dev

        lift_deg = abs(b_dev) / DUTY_PER_DEGREE

        status = ""
        if is_lifting:
            status = f"LIFTING ({lift_deg:.0f} deg)"
            lifting.append(leg_id)
        elif abs(a_dev) > 50:
            status = f"STEPPING (A={a_dev:+d})"
            stepping.append(leg_id)
        else:
            status = "home"

        print(f"  {info['name']:20s}  A={a_val:4d}({a_dev:+4d})  "
              f"B={b_val:4d}({b_dev:+4d})  C={c_val:4d}({c_dev:+4d})  "
              f"| {status}")

    # Summary
    if lifting:
        leg_names = [f"L{l}" for l in lifting]
        print(f"\n  >> LIFTING: {', '.join(leg_names)}")
        # Check tripod grouping
        if set(lifting) == {1, 3, 5}:
            print("     Tripod A (1,3,5) — rear-R, front-R, mid-L")
        elif set(lifting) == {2, 4, 6}:
            print("     Tripod B (2,4,6) — mid-R, front-L, rear-L")
    if stepping:
        leg_names = [f"L{l}" for l in stepping]
        print(f"  >> STEPPING: {', '.join(leg_names)}")

    return lifting, stepping


def main():
    # Find the .rob file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    rob_path = os.path.join(
        script_dir, "..", "..",
        "10. Action Group Files", "No.6 Obstacle Crossing.rob"
    )

    if not os.path.exists(rob_path):
        # Try alternate path
        rob_path = os.path.join(
            script_dir, "..",
            "10. Action Group Files", "No.6 Obstacle Crossing.rob"
        )

    if len(sys.argv) > 1:
        rob_path = sys.argv[1]

    if not os.path.exists(rob_path):
        print(f"ERROR: Cannot find .rob file at: {rob_path}")
        print("Usage: python3 decode_ag6.py [path/to/No.6 Obstacle Crossing.rob]")
        sys.exit(1)

    print(f"Reading: {rob_path}")
    frames = load_rob(rob_path)
    print(f"Total frames: {len(frames)}")

    print("\n" + "="*60)
    print("HOME B-SERVO VALUES (from frame 1):")
    print(f"  Right legs: {HOME_B_RIGHT}")
    print(f"  Left legs:  {HOME_B_LEFT}")
    print(f"  Lift convention: Right B<{HOME_B_RIGHT}=UP, Left B>{HOME_B_LEFT}=UP")
    print("="*60)

    all_lifts = []
    for i, frame in enumerate(frames):
        lifting, stepping = analyze_frame(i, frame["Servos"], frame["Duration"])
        all_lifts.append(lifting)

    # Pattern summary
    print("\n\n" + "="*60)
    print("PATTERN SUMMARY")
    print("="*60)
    print(f"{'Frame':>6s}  {'Duration':>8s}  {'Lifting':20s}  {'Type'}")
    print("-" * 60)
    for i, (frame, lifting) in enumerate(zip(frames, all_lifts)):
        if not lifting:
            ftype = "HOME / STEP"
        elif set(lifting) == {1, 3, 5}:
            ftype = "Tripod A LIFT"
        elif set(lifting) == {2, 4, 6}:
            ftype = "Tripod B LIFT"
        else:
            ftype = f"Custom: {lifting}"

        lift_str = ",".join(f"L{l}" for l in lifting) if lifting else "none"
        print(f"  {i+1:4d}  {frame['Duration']:6d}ms  {lift_str:20s}  {ftype}")

    # Key insight
    print("\n" + "="*60)
    print("KEY INSIGHT FOR CRATER MISSION")
    print("="*60)
    print("AG6 lifts ALL legs equally high (~83 deg B-servo deflection).")
    print("This is why it almost worked (high lift = clears obstacles)")
    print("but failed (rear legs also high = no traction on exit).")
    print()
    print("For crater: we need ASYMMETRIC lift:")
    print("  Front legs (3,4): full lift (reach into crater)")
    print("  Mid legs (2,5): moderate lift")
    print("  Rear legs (1,6): minimal lift (stay grounded for traction)")


if __name__ == "__main__":
    main()
