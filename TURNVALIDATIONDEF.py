import math
import time
import csv
import os
import nxt.locator
import nxt.motor

# ===== SETTINGS =====
WHEEL_DIAMETER_MM = 48.0
POWER = 70
CHECK_INTERVAL = 0.01

TARGET_ROBOT_DEG = 90.0
TURN_DIRECTION = "right"   # change to "right" when needed
CSV_FILENAME = "turn_validation_90deg.csv"

# Use your calibrated value here
TURN_WHEEL_DEG_PER_ROBOT_DEG = 2.549636

# ===== DERIVED =====
target_robot_deg = TARGET_ROBOT_DEG
target_wheel_deg = int(round(target_robot_deg * TURN_WHEEL_DEG_PER_ROBOT_DEG))


def get_next_trial_number(filename, direction):
    if not os.path.exists(filename):
        return 1

    max_trial = 0
    with open(filename, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["direction"] == direction:
                max_trial = max(max_trial, int(row["trial"]))
    return max_trial + 1


def append_row(filename, row, fieldnames):
    file_exists = os.path.exists(filename)
    with open(filename, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow(row)


def run_trial(brick, trial_num):
    """Execute one 90° in-place turn and return encoder data."""
    left_motor = nxt.motor.Motor(brick, nxt.motor.Port.B)
    right_motor = nxt.motor.Motor(brick, nxt.motor.Port.C)

    # Reset encoders
    left_motor.reset_position(False)
    right_motor.reset_position(False)
    time.sleep(0.1)

    print(f"\n--- Trial {trial_num} ---")
    print(f"Direction: {TURN_DIRECTION}")
    print(f"Attempting: {target_robot_deg}° robot turn")
    print(f"Commanding: {target_wheel_deg}° wheel rotation")
    print("Turning...")

    # In-place turn
    if TURN_DIRECTION == "left":
        left_motor.run(-POWER)
        right_motor.run(POWER)
    elif TURN_DIRECTION == "right":
        left_motor.run(POWER)
        right_motor.run(-POWER)
    else:
        raise ValueError("TURN_DIRECTION must be 'left' or 'right'")

    # Wait until target wheel rotation is reached
    try:
        while True:
            left_rot_abs = abs(left_motor.get_tacho().rotation_count)
            right_rot_abs = abs(right_motor.get_tacho().rotation_count)
            avg_rot_abs = (left_rot_abs + right_rot_abs) / 2.0

            if avg_rot_abs >= target_wheel_deg:
                break

            time.sleep(CHECK_INTERVAL)
    finally:
        left_motor.brake()
        right_motor.brake()

    # Let the robot fully settle
    time.sleep(0.3)

    # Final signed encoder values
    left_final = left_motor.get_tacho().rotation_count
    right_final = right_motor.get_tacho().rotation_count

    print(f"Left encoder:  {left_final}°")
    print(f"Right encoder: {right_final}°")

    return {
        "trial": trial_num,
        "direction": TURN_DIRECTION,
        "power": POWER,
        "target_robot_deg": target_robot_deg,
        "target_wheel_deg": target_wheel_deg,
        "left_encoder_deg": left_final,
        "right_encoder_deg": right_final,
    }


FIELDNAMES = [
    "trial",
    "direction",
    "power",
    "target_robot_deg",
    "target_wheel_deg",
    "left_encoder_deg",
    "right_encoder_deg",
]

with nxt.locator.find() as brick:
    print("=" * 60)
    print("TURN VALIDATION — 90 DEG")
    print("=" * 60)
    print(f"Connected to: {brick.get_device_info()[0]}")
    print(f"Wheel diameter: {WHEEL_DIAMETER_MM} mm")
    print(f"Power: {POWER}")
    print(f"Direction: {TURN_DIRECTION}")
    print(f"Target robot turn: {target_robot_deg}°")
    print(f"Target wheel rotation: {target_wheel_deg}°")
    print(f"CSV output: {CSV_FILENAME}")
    print()
    print("INSTRUCTIONS:")
    print("1. Align robot with the starting line")
    print("2. Use a 90° reference line on the floor")
    print("3. Run one trial")
    print("4. Measure overshoot/undershoot relative to the 90° line")
    print("5. Add measured deviation later in Excel")
    print()

    next_trial = get_next_trial_number(CSV_FILENAME, TURN_DIRECTION)

    while True:
        input(f"Press ENTER to run trial {next_trial} ({TURN_DIRECTION}), or Ctrl+C to stop...")

        row = run_trial(brick, next_trial)
        append_row(CSV_FILENAME, row, FIELDNAMES)

        print("Saved to CSV.")
        print("Now measure the deviation from the 90° reference line.")
        next_trial += 1