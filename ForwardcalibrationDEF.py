import math
import nxt.locator
import nxt.motor
from nxt.motor import SynchronizedMotors

# ===== SETTINGS =====
WHEEL_DIAMETER_MM = 48.0      # change if your wheel diameter is different
TARGET_DISTANCE_MM = 1000.0   # about 1 meter
POWER = 70
TIMEOUT = 10

# ===== CALCULATION =====
wheel_circumference_mm = math.pi * WHEEL_DIAMETER_MM
target_degrees = int((TARGET_DISTANCE_MM / wheel_circumference_mm) * 360)

with nxt.locator.find() as brick:
    print("Connected to:", brick.get_device_info()[0])

    left_motor = nxt.motor.Motor(brick, nxt.motor.Port.B)
    right_motor = nxt.motor.Motor(brick, nxt.motor.Port.C)
    sync_motors = SynchronizedMotors(left_motor, right_motor, 0)

    # Reset encoder counters before the run
    left_motor.reset_position(False)
    right_motor.reset_position(False)

    print(f"Target distance: {TARGET_DISTANCE_MM} mm")
    print(f"Wheel diameter: {WHEEL_DIAMETER_MM} mm")
    print(f"Target wheel rotation: {target_degrees} degrees")

    # Drive straight for the calculated encoder angle
    sync_motors.turn(POWER, target_degrees, brake=True, timeout=TIMEOUT)
    sync_motors.idle()

    # Read encoder values
    left_tacho = left_motor.get_tacho()
    right_tacho = right_motor.get_tacho()

    print("\nLEFT TACHO:")
    print(vars(left_tacho))

    print("\nRIGHT TACHO:")
    print(vars(right_tacho))