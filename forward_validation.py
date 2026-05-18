import math
import time
import csv
import nxt.locator
import nxt.motor
from nxt.motor import SynchronizedMotors

# =========================================
# Robot constants (calibrated)
# =========================================
WHEEL_DIAMETER = 48.0
WHEEL_CIRC = math.pi * WHEEL_DIAMETER
CORRECTION_FACTOR = 1.025535

POWER = 70
TARGET_DISTANCE_MM = 1000
TARGET_DEGREES = int(round((TARGET_DISTANCE_MM / WHEEL_CIRC) * 360))
NUM_TRIALS = 15

with nxt.locator.find() as brick:
    print("Connected:", brick.get_device_info()[0])

    motor_L = nxt.motor.Motor(brick, nxt.motor.Port.B)
    motor_R = nxt.motor.Motor(brick, nxt.motor.Port.C)
    sync = SynchronizedMotors(motor_L, motor_R, 0)

    results = []

    print(f"\nForward motion validation")
    print(f"Target distance: {TARGET_DISTANCE_MM} mm")
    print(f"Target encoder degrees: {TARGET_DEGREES}")
    print(f"Trials: {NUM_TRIALS}")
    print(f"Correction factor: {CORRECTION_FACTOR}")
    print()

    for trial in range(1, NUM_TRIALS + 1):
        input(f"Trial {trial}/{NUM_TRIALS} - Place robot on starting mark, press ENTER to drive...")

        # Reset encoders
        motor_L.reset_position(False)
        motor_R.reset_position(False)
        time.sleep(0.1)

        # Drive forward
        sync.run(POWER)

        while True:
            left_deg = abs(motor_L.get_tacho().rotation_count)
            right_deg = abs(motor_R.get_tacho().rotation_count)
            avg_deg = (left_deg + right_deg) / 2.0

            if avg_deg >= TARGET_DEGREES:
                break
            time.sleep(0.02)

        sync.idle()
        time.sleep(0.3)

        # Read final encoder values
        left_final = motor_L.get_tacho().rotation_count
        right_final = motor_R.get_tacho().rotation_count

        # Calculate distances
        left_mm = (left_final / 360.0) * WHEEL_CIRC
        right_mm = (right_final / 360.0) * WHEEL_CIRC
        avg_mm = (left_mm + right_mm) / 2.0
        corrected_mm = avg_mm * CORRECTION_FACTOR

        results.append({
            "trial": trial,
            "left_enc": left_final,
            "right_enc": right_final,
            "left_mm": round(left_mm, 2),
            "right_mm": round(right_mm, 2),
            "calculated_mm": round(avg_mm, 2),
            "corrected_mm": round(corrected_mm, 2),
            "measured_mm": ""
        })

        print(f"  Left encoder: {left_final}°  |  Right encoder: {right_final}°")
        print(f"  Calculated: {avg_mm:.2f} mm  |  Corrected: {corrected_mm:.2f} mm")
        print(f"  Measure the distance and note it down.\n")

    # Export to CSV
    filename = "forward_validation.csv"
    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Trial", "Left encoder (deg)", "Right encoder (deg)",
            "Left distance (mm)", "Right distance (mm)",
            "Calculated distance (mm)", "Corrected distance (mm)",
            "Measured distance (mm)"
        ])
        for r in results:
            writer.writerow([
                r["trial"], r["left_enc"], r["right_enc"],
                r["left_mm"], r["right_mm"],
                r["calculated_mm"], r["corrected_mm"],
                r["measured_mm"]
            ])

    print(f"Results saved to {filename}")
    print("Fill in the 'Measured distance' column manually after the test.")
