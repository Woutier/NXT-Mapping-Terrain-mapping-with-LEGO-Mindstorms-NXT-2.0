import math
import time
import nxt.locator
import nxt.motor
import nxt.sensor
import nxt.sensor.generic
from nxt.motor import SynchronizedMotors
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# =========================================
# Robot constants (calibrated)
# =========================================
WHEEL_DIAMETER = 48.0
WHEEL_CIRC = math.pi * WHEEL_DIAMETER
CORRECTION_FACTOR = 1.0255
TRACK_WIDTH = 122.382545
WHEEL_DEG_PER_ROBOT_DEG = TRACK_WIDTH / WHEEL_DIAMETER

POWER = 70
LOG_INTERVAL = 0.05
PLOT_INTERVAL = 0.10

# =========================================
# Mapping settings
# =========================================
OBSTACLE_CM = 30
SENSOR_CORRECTION = 0.5       # sensor reads ~2x the real distance
BOX_HALF = 800
BOUNDARY_MARGIN = 200
ESCAPE_TIME = 1.5


# Odometry
# =========================================
def encoder_to_mm(encoder_deg):
    return (encoder_deg / 360.0) * WHEEL_CIRC


def update_pose(x, y, theta, dL, dR):
    d_center = ((dL + dR) / 2.0) * CORRECTION_FACTOR
    d_theta = (dR - dL) / TRACK_WIDTH
    x += d_center * math.cos(theta + d_theta / 2.0)
    y += d_center * math.sin(theta + d_theta / 2.0)
    theta += d_theta
    return x, y, theta


# =========================================
# Plot setup
# =========================================
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))

traj_x = []
traj_y = []
obs_x = []
obs_y = []

line, = ax.plot([], [], linewidth=1.5, color="steelblue", label="Path")
pos_dot, = ax.plot([], [], 'bo', markersize=6, label="Robot")
obs_dots, = ax.plot([], [], 'ro', markersize=6, label="Obstacle")
arrow = None

boundary_rect = patches.Rectangle(
    (-BOX_HALF, -BOX_HALF), BOX_HALF * 2, BOX_HALF * 2,
    linewidth=2, edgecolor='black', facecolor='none', linestyle='--',
    label="Boundary"
)
ax.add_patch(boundary_rect)

ax.set_title("Boundary + Ultrasonic Test")
ax.set_xlabel("x (mm)")
ax.set_ylabel("y (mm)")
ax.set_xlim(-BOX_HALF - 200, BOX_HALF + 200)
ax.set_ylim(-BOX_HALF - 200, BOX_HALF + 200)
ax.axis("equal")
ax.grid(True, alpha=0.3)
ax.legend(loc="upper right")


def refresh_plot(x, y, theta):
    global arrow

    traj_x.append(x)
    traj_y.append(y)

    line.set_data(traj_x, traj_y)
    pos_dot.set_data([x], [y])
    obs_dots.set_data(obs_x, obs_y)

    if arrow is not None:
        arrow.remove()
    dx = 60 * math.cos(theta)
    dy = 60 * math.sin(theta)
    arrow = ax.arrow(x, y, dx, dy,
                     head_width=20, head_length=15, length_includes_head=True)

    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.001)


# Main
with nxt.locator.find() as brick:
    print("Connected:", brick.get_device_info()[0])

    motor_L = nxt.motor.Motor(brick, nxt.motor.Port.B)
    motor_R = nxt.motor.Motor(brick, nxt.motor.Port.C)
    sync = SynchronizedMotors(motor_L, motor_R, 0)

    us = brick.get_sensor(nxt.sensor.Port.S4, nxt.sensor.generic.Ultrasonic)

    x = 0.0
    y = 0.0
    theta = 0.0
    prev_L = motor_L.get_tacho().rotation_count
    prev_R = motor_R.get_tacho().rotation_count
    t_start = time.time()
    last_plot = 0.0


    def read_and_update(status):
        global x, y, theta, prev_L, prev_R, last_plot

        now_L = motor_L.get_tacho().rotation_count
        now_R = motor_R.get_tacho().rotation_count

        dL_mm = encoder_to_mm(now_L - prev_L)
        dR_mm = encoder_to_mm(now_R - prev_R)
        prev_L = now_L
        prev_R = now_R

        x, y, theta = update_pose(x, y, theta, dL_mm, dR_mm)

        now = time.time()
        if now - last_plot >= PLOT_INTERVAL:
            refresh_plot(x, y, theta)
            last_plot = now


    def stop():
        sync.idle()


    def get_distance():
        try:
            return us.get_sample()
        except Exception:
            return 255


    def turn(robot_deg, power=POWER):
        global prev_L, prev_R

        target_wheel_deg = int(round(abs(robot_deg) * WHEEL_DEG_PER_ROBOT_DEG))

        motor_L.reset_position(False)
        motor_R.reset_position(False)
        prev_L = 0
        prev_R = 0

        if robot_deg > 0:
            motor_L.run(-power)
            motor_R.run(power)
        elif robot_deg < 0:
            motor_L.run(power)
            motor_R.run(-power)
        else:
            return

        while True:
            avg = (abs(motor_L.get_tacho().rotation_count)
                   + abs(motor_R.get_tacho().rotation_count)) / 2.0
            if avg >= target_wheel_deg:
                break
            read_and_update("turning")
            time.sleep(LOG_INTERVAL)

        motor_L.idle()
        motor_R.idle()
        read_and_update("stopped")


    def drive_for(duration, check_boundary=True, check_obstacle=True):
        """Drive straight. Stops early on boundary or obstacle.
        Returns: "boundary", "obstacle", or "done"."""
        sync.run(POWER)
        start = time.time()

        while time.time() - start < duration:
            if check_obstacle:
                dist = get_distance()
                if dist <= OBSTACLE_CM:
                    stop()
                    read_and_update("obstacle")
                    return "obstacle", dist

            if check_boundary and is_near_boundary():
                stop()
                read_and_update("boundary")
                return "boundary", 0

            read_and_update("driving")
            time.sleep(LOG_INTERVAL)

        stop()
        read_and_update("stopped")
        return "done", 0


    def is_near_boundary():
        if abs(x) > BOX_HALF - BOUNDARY_MARGIN:
            return True
        if abs(y) > BOX_HALF - BOUNDARY_MARGIN:
            return True
        return False


    def turn_toward_center():
        angle_to_center = math.atan2(-y, -x)
        turn_needed = angle_to_center - theta

        while turn_needed > math.pi:
            turn_needed -= 2 * math.pi
        while turn_needed < -math.pi:
            turn_needed += 2 * math.pi

        turn(math.degrees(turn_needed))
        print(f"Boundary at ({x:.0f}, {y:.0f}), turned toward center")


    def mark_obstacle(dist_cm):
        real_dist_mm = dist_cm * 10 * SENSOR_CORRECTION
        ox = x + real_dist_mm * math.cos(theta)
        oy = y + real_dist_mm * math.sin(theta)
        obs_x.append(ox)
        obs_y.append(oy)
        print(f"Obstacle at ({ox:.0f}, {oy:.0f})")


    # =========================================
    # Mission: just drive straight, react only
    # to boundaries and obstacles
    # =========================================
    print("Starting boundary + ultrasonic test")
    print(f"Box: {BOX_HALF*2}mm x {BOX_HALF*2}mm")
    print(f"Obstacle threshold: {OBSTACLE_CM} cm")
    print("Press Ctrl+C to stop")
    print()

    refresh_plot(x, y, theta)

    try:
        while True:
            # just drive forward for a long time (30s)
            # the only thing that stops the robot is boundary or obstacle
            result, dist = drive_for(30.0, check_boundary=True, check_obstacle=True)

            if result == "obstacle":
                mark_obstacle(dist)
                # simple reaction: turn 90 degrees left and keep going
                turn(90)
                print("Obstacle detected, turned left 90")
                input("Press ENTER to continue...")

            elif result == "boundary":
                turn_toward_center()
                print("Ready to escape from boundary")
                input("Press ENTER to continue...")
                # escape drive: get away from edge before checking again
                drive_for(ESCAPE_TIME, check_boundary=False, check_obstacle=False)

            time.sleep(0.1)

    except KeyboardInterrupt:
        stop()
        print("\nStopped by user")

    stop()

    print(f"\nDone. Obstacles found: {len(obs_x)}")
    print("\nObstacle positions:")
    for i in range(len(obs_x)):
        print(f"  ({obs_x[i]:.0f}, {obs_y[i]:.0f})")

    print("\nClose the plot to exit.")
    plt.ioff()
    plt.show()
