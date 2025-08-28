# Team Toolkit – Single File (SPIKE App 3.5)
# Drive: A (left), E (right)
# Wheels: 87 mm diameter
# Track width: 143 mm (center-to-center distance between wheels)

from hub import port, motion_sensor
import runloop
import motor_pair
import motor

from math import pi

# -----------------------------
# Robot configuration
# -----------------------------
PAIR = motor_pair.PAIR_1

# Main Drive motors
LEFT_DRIVE= port.A
RIGHT_DRIVE = port.E

# Downward color sensors
left_color = port.F
right_color = port.B

# Attachment/Tool motors
attachment1 = port.C
attachment2 = port.D

# Wheel and robot dimensions
WHEEL_D_MM= 88.0    # wheel diameter
TRACK_W_MM= 143.0    # track width
ACCEL    = 1000    # deg/s^2
DECEL    = 1000

# -----------------------------
# Geometry helpers
# -----------------------------
def _cm_to_deg(cm: float) -> int:
    """Convert straight-line distance (cm) to motor shaft degrees."""
    circ_mm = pi * WHEEL_D_MM                    # wheel circumference (mm)
    rotations = (abs(cm) * 10.0) / circ_mm    # cm -> mm -> rotations
    return int(round(rotations * 360.0))        # -> degrees

def _robot_deg_to_wheel_deg(robot_deg: float) -> int:
    """Convert a robot in-place rotation (degrees) to wheel shaft degrees."""
    turn_circ_mm = pi * TRACK_W_MM                        # turn circle (mm)
    travel_mm = (abs(robot_deg) / 360.0) * turn_circ_mm    # wheel path (mm)
    wheel_rot = travel_mm / (pi * WHEEL_D_MM)
    return int(round(wheel_rot * 360.0))

def _yaw_deg() -> float:
    y_decideg, _, _ = motion_sensor.tilt_angles()
    return y_decideg / 10.0

def _reset_yaw(deg: int = 0) -> None:
    motion_sensor.reset_yaw(int(deg * 10))
# -----------------------------
# Init
# -----------------------------

async def reset_yaw():
    motion_sensor.reset_yaw(0)
    motor.reset_relative_position(LEFT_DRIVE, 0)
    motor.reset_relative_position(RIGHT_DRIVE, 0)
    await runloop.sleep_ms(500)

async def init_robot(default_speed: int = 500):
    """
    Pair A/E as the drive motors and set a default speed for the pair.
    Call this once at the start of your program.
    """
    motor_pair.pair(PAIR, LEFT_DRIVE, RIGHT_DRIVE)
    await reset_yaw()


# -----------------------------
# Core movement
# -----------------------------
async def drive_cm(cm: float,
                velocity: int = 500,
                stop_mode: int = motor.BRAKE,
                acceleration: int = ACCEL,
                deceleration: int = DECEL):
    """
    Drive straight for 'cm' centimeters using encoders.
    +cm = forward, -cm = backward
    """
    deg = _cm_to_deg(cm)
    vel = velocity if cm >= 0 else -velocity
    await motor_pair.move_for_degrees(
        PAIR,
        deg,
        0,                            # steering = 0 → straight
        velocity=vel,
        stop=stop_mode,
        acceleration=acceleration,
        deceleration=deceleration
    )

# -----------------------------
# Drive with gyro correction
# -----------------------------

async def drive_cm_gyro(cm: float,
                        velocity: int = 420,
                        step_cm: float = 8.0,    # bigger chunks = fewer starts/stops
                        kp: float = 0.8,        # gentle gain for smoothness
                        steer_limit: int = 20,    # cap steering authority
                        deadband_deg: float = 1.0,
                        steer_rate_limit: int = 6,# max change in steer per chunk
                        stop_mode_end: int = motor.BRAKE):
    """
    Smooth encoder-stepped straight drive with yaw correction.
    Keeps motion continuous between chunks (CONTINUE) and eases steering.
    +cm forward, -cm backward.
    """
    total_deg = _cm_to_deg(cm)
    remaining = abs(total_deg)
    dir_sign= 1 if cm >= 0 else -1
    step_deg= max(60, _cm_to_deg(step_cm))# ensure meaningful chunk

    # gyro prep
    _reset_yaw(0)

    # steering state for rate limiting + low-pass yaw
    prev_steer = 0
    yaw_filt = _yaw_deg()

    while remaining > 0:
        chunk = step_deg if remaining > step_deg else remaining

        # --- Read & filter yaw (low-pass to avoid noise)
        yaw_now = _yaw_deg()
        yaw_filt = 0.8 * yaw_filt + 0.2 * yaw_now

        # --- Compute steering for this chunk
        err = -yaw_filt# want 0°
        if abs(err) < deadband_deg:
            raw_steer = 0
        else:
            raw_steer = int(kp * err)
            raw_steer = max(-steer_limit, min(steer_limit, raw_steer))

        # --- Ease steering change (rate limit)
        delta = raw_steer - prev_steer
        if delta > steer_rate_limit:
            steer = prev_steer + steer_rate_limit
        elif delta < -steer_rate_limit:
            steer = prev_steer - steer_rate_limit
        else:
            steer = raw_steer
        prev_steer = steer

        # --- Execute chunk without stopping (CONTINUE keeps motion smooth)
        await motor_pair.move_for_degrees(
            PAIR,
            chunk,
            steer,
            velocity=dir_sign * velocity,
            stop=motor.CONTINUE,    # <— keep rolling into next chunk
            acceleration=ACCEL,
            deceleration=DECEL // 2# softer decel to reduce jerk
        )
        remaining -= chunk
        print(remaining)

    # Final, tidy stop at the end of the whole move
    motor_pair.stop(PAIR, stop=stop_mode_end)


async def turn_deg(angle_deg: float,
                velocity: int = 400,
                stop_mode: int = motor.BRAKE,
                acceleration: int = ACCEL,
                deceleration: int = DECEL):
    """
    In-place pivot by robot degrees (positive = one way, negative = the other).
    Uses track width to compute wheel rotation.
    """
    deg = _robot_deg_to_wheel_deg(angle_deg)
    steering = 100 if angle_deg > 0 else -100    # ±100 = spin in place
    await motor_pair.move_for_degrees(
        PAIR,
        deg,
        steering,
        velocity=velocity,
        stop=stop_mode,
        acceleration=acceleration,
        deceleration=deceleration
    )
    await reset_yaw()




async def arc_turn(radius_cm: float,
                angle_deg: float,
                velocity: int = 400,
                stop_mode: int = motor.BRAKE):
    """
    Smooth arc using constant steering (approximate).
    For precise arcs, use per-wheel degrees with motor.run_for_degrees.
    """
    if radius_cm <= 0:
        return
    # Steering approximation: s ≈ (track / (2R)) * 100 (clip to [-100, 100])
    s = int(max(-100, min(100, (TRACK_W_MM / (2.0 * (radius_cm * 10.0))) * 100.0)))
    # Arc length ≈ R * theta
    arc_len_cm = abs(radius_cm * (angle_deg * pi / 180.0))
    deg = _cm_to_deg(arc_len_cm)
    steering = s if angle_deg > 0 else -s
    await motor_pair.move_for_degrees(PAIR, deg, steering, velocity=velocity, stop=stop_mode)


# -----------------------------
# Attachments (examples)
# -----------------------------
async def run_attachment_deg(which_port,
                            degrees: int,
                            velocity: int = 600,
                            stop_mode: int = motor.BRAKE):
    """Run an attachment motor by degrees (C or D typically)."""
    await motor.run_for_degrees(which_port, degrees, velocity=velocity, stop=stop_mode)

async def timed_attachment(which_port,
                        velocity: int = 400,
                        ms: int = 250,
                        stop_mode: int = motor.BRAKE):
    """Run an attachment motor for a fixed time (ms)."""
    await motor.run_for_time(which_port, ms, velocity=velocity, stop=stop_mode)

# -----------------------------
# Micro moves
# -----------------------------
async def nudge_cm(cm: float = 1.5, velocity: int = 250):
    """Small forward/backward bump to settle into models."""
    await drive_cm(cm, velocity=velocity)

async def micro_turn_deg(angle: float = 3.0, velocity: int = 200):
    """Tiny heading adjustment."""
    await turn_deg(angle, velocity=velocity)

# -----------------------------
# Main program
# -----------------------------
async def main():
    await init_robot(default_speed=500)

    # Example: 60 cm forward, 90° turn 4 times - create a square
    for _ in range(4):
        # Move forward 
        await drive_cm(60, velocity=200)
        # Turn
        await turn_deg(90, velocity=200)

# Run the main loop
runloop.run(main())