import math
import time

from adafruit_servokit import ServoKit
from robotKinematics_legacy import RobotKinematics


def clamp(value, lower=19, upper=90):
    return max(lower, min(value, upper))


class LegacyRobotController:
    def __init__(self, model, lp=7.125, l1=6.20, l2=4.50, lb=4.00, debug=False):
        self.robot = model
        self.debug = debug

        self.Controller = ServoKit(channels=16)
        self.s1 = self.Controller.servo[13]
        self.s2 = self.Controller.servo[15]
        self.s3 = self.Controller.servo[14]

        for servo in (self.s1, self.s2, self.s3):
            servo.actuation_range = 270
            servo.set_pulse_width_range(500, 2500)

        self.initialize()

    def initialize(self):
        print("Initializing legacy controller ...")
        self.set_motor_angles(54, 54, 54)
        self.interpolate_time([19, 19, 19], duration=0.25)
        time.sleep(1)
        self.Goto_time_spherical(0, 0, 8.26, t=0.25)
        time.sleep(1)
        print("Initialized!")

    def set_motor_angles(self, theta1, theta2, theta3):
        self.s1.angle = clamp(theta1) - 4
        self.s2.angle = clamp(theta2)
        self.s3.angle = clamp(theta3)
        if self.debug:
            print(f"Set angles: {self.s1.angle}, {self.s2.angle}, {self.s3.angle}")

    def interpolate_time(self, target_angles, steps=100, duration=0.3, individual_durations=None):
        current_angles = [self.s1.angle, self.s2.angle, self.s3.angle]
        if individual_durations is None:
            individual_durations = [duration] * 3
        max_duration = max(individual_durations)
        steps = max(1, int(max_duration / 0.01))
        for i in range(steps + 1):
            t = i * max_duration / steps
            angles = [
                c + (t_angle - c) * min(t / d, 1) if d > 0 else t_angle
                for c, t_angle, d in zip(current_angles, target_angles, individual_durations)
            ]
            self.set_motor_angles(*angles)
            time.sleep(max_duration / steps)

    def Goto_time_spherical(self, theta, phi, h, t=0.5):
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        target_angles = [
            math.degrees(math.pi * 0.5 - self.robot.theta1),
            math.degrees(math.pi * 0.5 - self.robot.theta2),
            math.degrees(math.pi * 0.5 - self.robot.theta3),
        ]
        self.interpolate_time(target_angles, duration=t)

    def Goto_N_time_spherical(self, theta, phi, h):
        self.robot.solve_inverse_kinematics_spherical(theta, phi, h)
        target_angles = [
            math.degrees(math.pi * 0.5 - self.robot.theta1),
            math.degrees(math.pi * 0.5 - self.robot.theta2),
            math.degrees(math.pi * 0.5 - self.robot.theta3),
        ]
        self.set_motor_angles(*target_angles)


if __name__ == "__main__":
    model = RobotKinematics()
    controller = LegacyRobotController(model)
    time.sleep(0.5)
