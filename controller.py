import math
import time

import robotKinematics as rk
from imu import MPU6050

try:
    from adafruit_servokit import ServoKit
except ImportError:  # pragma: no cover - hardware dependency
    ServoKit = None


def clamp(value, lower=19, upper=89):
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
        neutral_angle=54.0,
        neutral_blend=1.0,
    ):
        if ServoKit is None:
            raise ImportError(
                "adafruit_servokit is required to drive the servos. Install it on the Raspberry Pi."
            )

        self.robot = robot
        self.offsets = servo_offsets
        self.neutral_angle = neutral_angle
        self.neutral_blend = max(0.0, min(neutral_blend, 1.0))
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
        neutral = self.neutral_angle
        blend = self.neutral_blend

        def apply(theta, offset):
            base = clamp(theta)
            if blend < 1.0:
                base = neutral + (base - neutral) * blend
            return base + offset

        self.s1.angle = apply(theta1, o1)
        self.s2.angle = apply(theta2, o2)
        self.s3.angle = apply(theta3, o3)

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
    def __init__(self, kp, ki, kd, max_out=15.0, integral_decay=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_err = 0.0
        self.decay = integral_decay

    def update(self, error, dt):
        if dt <= 0:
            dt = 1e-3

        # Apply optional decay to bleed the integrator toward zero.
        if self.decay > 0:
            decay_factor = max(0.0, 1.0 - self.decay * dt)
            self.integral *= decay_factor

        proposed_integral = self.integral + error * dt
        derivative = (error - self.prev_err) / dt
        self.prev_err = error

        out = self.kp * error + self.ki * proposed_integral + self.kd * derivative
        clamped = max(min(out, self.max_out), -self.max_out)

        # Basic anti-windup: only accept the new integral when not saturated.
        if clamped == out:
            self.integral = proposed_integral

        return clamped


class TiltController:
    """
    Controller for 3-RRS plate using MPU6050.
    Servo objects come from original RobotController.
    """

    def __init__(
        self,
        robot_controller,
        debug=False,
        axis_rotation_deg=0.0,
        invert_pitch=False,
        invert_roll=False,
        pitch_gain=1.0,
        roll_gain=1.0,
        pid_kp=0.9,
        pid_ki=0.0,
        pid_kd=0.03,
        pid_integral_decay=0.0,
        pid_max_out=15.0,
        output_gain=1.0,
        tilt_limit_deg=15.0,
        pitch_offset=0.0,
        roll_offset=0.0,
    ):
        self.robot = robot_controller
        self.s1 = self.robot.s1
        self.s2 = self.robot.s2
        self.s3 = self.robot.s3

        self.pid_x = PID(pid_kp, pid_ki, pid_kd, max_out=pid_max_out, integral_decay=pid_integral_decay)
        self.pid_y = PID(pid_kp, pid_ki, pid_kd, max_out=pid_max_out, integral_decay=pid_integral_decay)

        # Instantiate MPU6050 here
        self.imu = MPU6050()

        self.last_time = time.time()
        self.debug = debug
        self.output_gain = output_gain
        self.axis_rotation_rad = math.radians(axis_rotation_deg)
        self.axis_cos = math.cos(self.axis_rotation_rad)
        self.axis_sin = math.sin(self.axis_rotation_rad)
        self.invert_pitch = invert_pitch
        self.invert_roll = invert_roll
        self.pitch_gain = pitch_gain
        self.roll_gain = roll_gain
        self.tilt_limit = abs(tilt_limit_deg)
        self.pitch_offset = pitch_offset
        self.roll_offset = roll_offset

    def _transform_axes(self, pitch, roll):
        pitch -= self.pitch_offset
        roll -= self.roll_offset
        rotated_pitch = pitch * self.axis_cos - roll * self.axis_sin
        rotated_roll = pitch * self.axis_sin + roll * self.axis_cos
        if self.invert_pitch:
            rotated_pitch = -rotated_pitch
        if self.invert_roll:
            rotated_roll = -rotated_roll
        rotated_pitch *= self.pitch_gain
        rotated_roll *= self.roll_gain
        return rotated_pitch, rotated_roll

    def update(self):
        """
        Reads MPU6050 internally, computes tilt correction, applies to servos
        """
        t = time.time()
        dt = t - self.last_time
        self.last_time = t

        # Read pitch/roll directly from IMU
        raw_pitch, raw_roll = self.imu.read()
        pitch, roll = self._transform_axes(raw_pitch, raw_roll)

        # Compute PID corrections
        corr_x = self.output_gain * self.pid_x.update(-pitch, dt)
        corr_y = self.output_gain * self.pid_y.update(-roll, dt)

        if self.tilt_limit > 0:
            corr_x = max(min(corr_x, self.tilt_limit), -self.tilt_limit)
            corr_y = max(min(corr_y, self.tilt_limit), -self.tilt_limit)

        # Map to servo angles
        theta1, theta2, theta3 = rk.tilt_to_servos(corr_x, corr_y)

        # Apply angles
        self.s1.angle = theta1
        self.s2.angle = theta2
        self.s3.angle = theta3

        if self.debug:
            print(
                f"raw=({raw_pitch:+6.2f}, {raw_roll:+6.2f}) "
                f"rot=({pitch:+6.2f}, {roll:+6.2f}) | "
                f"PID=({corr_x:+6.2f}, {corr_y:+6.2f}) | "
                f"servos=({theta1:5.1f}, {theta2:5.1f}, {theta3:5.1f})",
                flush=True,
            )
