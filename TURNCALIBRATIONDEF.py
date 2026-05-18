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
N_ROTATIONS = 4
TURN_WHEEL_DEG_PER_ROBOT_DEG_GUESS = 224.0 / 90.0
TURN_DIRECTION = "right"   # change to "right" when needed
CSV_FILENAME = "turn_calibration_raw.csv"

# ===== DERIVED =====
wheel_circumference_mm = math.pi * WHEEL_DIAMETER_MM
target_robot_deg = N_ROTATIONS * 360.0
target_wheel_deg = int(round(target_robot_deg * TURN_WHEEL_DEG_PER_ROBOT_DEG_GUESS))


def deg_to_mm_raw(encoder_deg):
    return (encoder_deg / 360.0) * wheel_circumference_mm


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
    left_motor = nxt.motor.Motor(brick, nxt.motor.Port.B)
    right_motor = nxt.motor.Motor(brick, nxt.motor.Port.C)

    left_motor.reset_position(False)
    right_motor.reset_position(False)
    time.sleep(0.1)

    print(f"\n--- Trial {trial_num} ---")
    print(f"Direction: {TURN_DIRECTION}")
    print(f"Attempting: {N_ROTATIONS} full rotations ({target_robot_deg}° robot)")
    print(f"Commanding: {target_wheel_deg}° wheel rotation")
    print("Turning...")

    if TURN_DIRECTION == "right":
        left_motor.run(-POWER)
        right_motor.run(POWER)
    else:
        left_motor.run(POWER)
        right_motor.run(-POWER)

    try:
        while True:
            left_rot_abs = abs(left_motor.get_tacho().rotation_count)
            right_rot_abs = abs(right_motor.get_tacho().rotation_count)
            avg_rot_abs = (left_rot_abs + right_rot_abs) / 2.0

            if avg_rot_abs >= target_wheel_deg:
                break

            time.sleep(CHECK_INTERVAL)
    finally:
        left_motor.idle()
        right_motor.idle()

    time.sleep(0.3)

    left_final = left_motor.get_tacho().rotation_count
    right_final = right_motor.get_tacho().rotation_count

    d_left_mm = deg_to_mm_raw(left_final)
    d_right_mm = deg_to_mm_raw(right_final)
    abs_diff_mm = abs(d_left_mm - d_right_mm)

    print(f"Left encoder:  {left_final}° → {d_left_mm:.2f} mm")
    print(f"Right encoder: {right_final}° → {d_right_mm:.2f} mm")
    print(f"|d_left - d_right| = {abs_diff_mm:.2f} mm")

    return {
        "trial": trial_num,
        "direction": TURN_DIRECTION,
        "power": POWER,
        "n_rotations": N_ROTATIONS,
        "target_robot_deg": round(target_robot_deg, 1),
        "target_wheel_deg": target_wheel_deg,
        "left_encoder_deg": left_final,
        "right_encoder_deg": right_final,
        "d_left_mm": round(d_left_mm, 2),
        "d_right_mm": round(d_right_mm, 2),
        "abs_diff_mm": round(abs_diff_mm, 2),
    }


FIELDNAMES = [
    "trial",
    "direction",
    "power",
    "n_rotations",
    "target_robot_deg",
    "target_wheel_deg",
    "left_encoder_deg",
    "right_encoder_deg",
    "d_left_mm",
    "d_right_mm",
    "abs_diff_mm",
]

with nxt.locator.find() as brick:
    print("=" * 60)
    print("TURN CALIBRATION — RAW DATA LOGGER")
    print("=" * 60)
    print(f"Connected to: {brick.get_device_info()[0]}")
    print(f"Direction: {TURN_DIRECTION}")
    print(f"CSV output: {CSV_FILENAME}")
    print()

    next_trial = get_next_trial_number(CSV_FILENAME, TURN_DIRECTION)

    while True:
        input(f"Press ENTER to run trial {next_trial} ({TURN_DIRECTION}), or Ctrl+C to stop...")

        row = run_trial(brick, next_trial)
        append_row(CSV_FILENAME, row, FIELDNAMES)

        print("Saved to CSV.")
        print("Mark the final heading and measure later in Excel.")
        next_trial += 1