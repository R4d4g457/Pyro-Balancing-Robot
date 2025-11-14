import math
import time

import robotKinematics as rk
from imu import MPU6050

try:
    from adafruit_servokit import ServoKit
except ImportError:  # pragma: no cover - hardware dependency
    ServoKit = None


def clamp(value, lower=19, upper=90):
    """Clamp servo angles to the physically safe motion range."""
    return max(lower, min(value, upper))


class RobotController:
    """Hardware interface responsible for configuring and commanding the servos."""

    def __init__(
        self,
        robot=None,
        servo_channels=(13, 15, 14),
        servo_offsets=(0.0, 0.0, 0.0),
        pulse_range=(500, 2500),
        actuation_range=270,
    ):
        if ServoKit is None:
            raise ImportError(
                "adafruit_servokit is required to drive the servos. Install it on the Raspberry Pi."
            )

        self.robot = robot
        self.offsets = servo_offsets
        self.Controller = ServoKit(channels=16)

        ch1, ch2, ch3 = servo_channels
        self.s1 = self.Controller.servo[ch1]
        self.s2 = self.Controller.servo[ch2]
        self.s3 = self.Controller.servo[ch3]

        for servo in (self.s1, self.s2, self.s3):
            servo.actuation_range = actuation_range
            servo.set_pulse_width_range(*pulse_range)

        self.initialize()

    def initialize(self):
        """Move to a safe neutral pose on boot."""
        self.set_motor_angles(54, 54, 54)
        time.sleep(0.5)

    def set_motor_angles(self, theta1, theta2, theta3):
        """Apply calibrated servo angles after clamping into a safe range."""
        o1, o2, o3 = self.offsets
        self.s1.angle = clamp(theta1) + o1
        self.s2.angle = clamp(theta2) + o2
        self.s3.angle = clamp(theta3) + o3

    def interpolate_time(self, target_angles, duration=0.3, steps=100):
        """Linearly interpolate servo angles over a duration."""
        current_angles = [self.s1.angle, self.s2.angle, self.s3.angle]
        duration = max(duration, 0.01)
        steps = max(1, steps)
        for i in range(steps + 1):
            t = i / steps
            interp = [c + (ta - c) * t for c, ta in zip(current_angles, target_angles)]
            self.set_motor_angles(*interp)
            time.sleep(duration / steps)

    # Legacy helpers retained for compatibility with the simulator tooling.
    def _require_robot_model(self):
        if self.robot is None:
            raise RuntimeError("RobotController was initialised without a robot model.")

    def goto_time_vector(self, a, b, c, h, t=0.5):
        self._require_robot_model()
        self.robot.solve_inverse_kinematics_vector(a, b, c, h)
        target = [
            math.degrees(math.pi * 0.5 - self.robot.theta1),
            math.degrees(math.pi * 0.5 - self.robot.theta2),
            math.degrees(math.pi * 0.5 - self.robot.theta3),
        ]
        self.interpolate_time(target, duration=t)

    def goto_time_spherical(self, theta, phi, h, t=0.5):
        self._require_robot_model()
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        target = [
            math.degrees(math.pi * 0.5 - self.robot.theta1),
            math.degrees(math.pi * 0.5 - self.robot.theta2),
            math.degrees(math.pi * 0.5 - self.robot.theta3),
        ]
        self.interpolate_time(target, duration=t)


class PID:
    def __init__(self, kp, ki, kd, max_out=15.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_err) / dt if dt > 0 else 0.0
        self.prev_err = error

        out = self.kp * error + self.ki * self.integral + self.kd * derivative

        return max(min(out, self.max_out), -self.max_out)


class TiltController:
    """
    Controller for 3-RRS plate using MPU6050.
    Servo objects come from original RobotController.
    """

    def __init__(self, robot_controller):
        self.robot = robot_controller
        self.s1 = self.robot.s1
        self.s2 = self.robot.s2
        self.s3 = self.robot.s3

        self.pid_x = PID(0.9, 0.0, 0.03)
        self.pid_y = PID(0.9, 0.0, 0.03)

        # Instantiate MPU6050 here
        self.imu = MPU6050()

        # Track last update time
        import time

        self.last_time = time.time()

    def update(self):
        """
        Reads MPU6050 internally, computes tilt correction, applies to servos
        """
        import time

        t = time.time()
        dt = t - self.last_time
        self.last_time = t

        # Read pitch/roll directly from IMU
        pitch, roll = self.imu.read()

        # Compute PID corrections
        corr_x = self.pid_x.update(-pitch, dt)
        corr_y = self.pid_y.update(-roll, dt)

        # Map to servo angles
        theta1, theta2, theta3 = rk.tilt_to_servos(corr_x, corr_y)

        # Apply angles
        self.s1.angle = theta1
        self.s2.angle = theta2
        self.s3.angle = theta3
