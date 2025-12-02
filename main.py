import os
import time
from controller import RobotController  # Original class that creates s1, s2, s3
from controller import TiltController  # Updated tilt controller with MPU6050


def _env_bool(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _env_float(name, default):
    value = os.environ.get(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def main():
    # Instantiate the original robot controller (sets up servos)
    robot = RobotController()

    # Create the tilt controller, passing the robot instance
    debug = os.environ.get("TILT_DEBUG", "").lower() not in ("", "0", "false", "no")
    axis_rotation = _env_float("PYRO_AXIS_ROT_DEG", 0.0)
    pitch_gain = _env_float("PYRO_PITCH_GAIN", 1.0)
    roll_gain = _env_float("PYRO_ROLL_GAIN", 1.0)
    invert_pitch = _env_bool("PYRO_INVERT_PITCH", False)
    invert_roll = _env_bool("PYRO_INVERT_ROLL", False)

    controller = TiltController(
        robot,
        debug=debug,
        axis_rotation_deg=axis_rotation,
        invert_pitch=invert_pitch,
        invert_roll=invert_roll,
        pitch_gain=pitch_gain,
        roll_gain=roll_gain,
    )
    if debug:
        print("TiltController debug logging enabled (TILT_DEBUG set).", flush=True)

    dt = 0.01  # Loop period (~100 Hz)
    time.sleep(1)  # Allow IMU to stabilise

    while True:
        # Controller reads MPU6050 internally, computes PID, updates servos
        controller.update()

        # Maintain loop rate
        time.sleep(dt)


if __name__ == "__main__":
    main()
