# Team Toolkit – Single File (SPIKE App 3.5)
# Drive: A (left), E (right)
# Wheels: 87 mm diameter
# Track width: 143 mm (center-to-center distance between wheels)

from hub import port
import runloop
import motor_pair
import motor

from math import pi

# -----------------------------
# Robot configuration
# -----------------------------
PAIR = motor_pair.PAIR_1
LEFT_DRIVE= port.A
RIGHT_DRIVE = port.E

WHEEL_D_MM= 87.0    # your wheels
TRACK_W_MM= 143.0    # your track width
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

# -----------------------------
# Init
# -----------------------------
async def init_robot(default_speed: int = 500):
    """
    Pair A/E as the drive motors and set a default speed for the pair.
    Call this once at the start of your program.
    """
    motor_pair.pair(PAIR, LEFT_DRIVE, RIGHT_DRIVE)

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
# Demo (remove or edit as needed)
# -----------------------------
async def main():
    await init_robot(default_speed=500)

    # Example: 60 cm forward, 90° turn, 60 cm forward, then back 60 cm
    await drive_cm(60, velocity=200)
    await turn_deg(90, velocity=400)

    await drive_cm(60, velocity=200)
    await turn_deg(90, velocity=400)

    await drive_cm(60, velocity=200)
    await turn_deg(90, velocity=400)

    await drive_cm(60, velocity=200)
    await turn_deg(90, velocity=400)




runloop.run(main())