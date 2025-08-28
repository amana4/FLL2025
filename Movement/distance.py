from hub import port
import runloop
import motor_pair
import motor
from math import pi

# --- Robot constants ---
WHEEL_DIAMETER_MM = 87.0    # wheel size
GEAR_RATIO = 1.0            # adjust if gearing is added
PAIR = motor_pair.PAIR_1    # our motor pair slot

def cm_to_degrees(cm: float) -> int:
    """Convert centimeters to motor degrees."""
    wheel_circ_mm = pi * WHEEL_DIAMETER_MM
    wheel_rotations = (cm * 10.0) / wheel_circ_mm
    motor_rotations = wheel_rotations / GEAR_RATIO
    return int(round(motor_rotations * 360.0))

async def drive_cm(cm: float,
                velocity_dps: int = 360,
                stop_mode: int = motor.BRAKE,
                acceleration: int = 1000,
                deceleration: int = 1000):
    """
    Drive straight for 'cm' centimeters.
    Positive = forward, Negative = backward.
    """
    degrees = cm_to_degrees(abs(cm))
    vel = velocity_dps if cm >= 0 else -velocity_dps

    # Move motors as a pair (steering=0 for straight)
    await motor_pair.move_for_degrees(
        PAIR,
        degrees,
        0,
        velocity=vel,
        stop=stop_mode,
        acceleration=acceleration,
        deceleration=deceleration
    )

async def main():
    # Pair once at start (A = left, E = right)
    motor_pair.pair(PAIR, port.A, port.E)

    # Drive forward 100 cm
    await drive_cm(60.96, velocity_dps=300)

    await runloop.sleep_ms(1000)

    # Drive backward 100 cm
    await drive_cm(-60.96, velocity_dps=300)

runloop.run(main())