#!/usr/bin/env python3
"""Drive the three servos through a simple "Mexican wave" pattern."""

import time

from adafruit_servokit import ServoKit

# Physical wiring from wiring.md matches these PCA9685 channels.
SERVO_CHANNELS = {"A": 13, "B": 15, "C": 14}
CLAMP_DEG = (19, 90)
TRANSITION_TIME = 0.5  # seconds per pose-to-pose move
TRANSITION_STEPS = 60  # higher = smoother

# Each tuple holds (description, {servo_name: angle, ...})
WAVE_STEPS = (
    ("A+B high", {"A": 90, "B": 90, "C": 19}),
    ("B+C high", {"A": 19, "B": 90, "C": 90}),
    ("C+A high", {"A": 90, "B": 19, "C": 90}),
)


def clamp(angle: float) -> float:
    """Clip an angle to the safe servo limits."""
    lo, hi = CLAMP_DEG
    return max(lo, min(hi, angle))


def configure_servos(servos):
    """Apply actuation/pulse settings that match controller.py."""
    for servo in servos.values():
        servo.actuation_range = 270
        servo.set_pulse_width_range(500, 2500)


def interpolate_move(servos, start, target, duration, steps):
    """Linearly sweep servos from start to target angles."""
    if steps <= 0:
        steps = 1
    dt = max(duration / steps, 0.001)
    for i in range(1, steps + 1):
        blend = i / steps
        for name, servo in servos.items():
            angle = start[name] + (target[name] - start[name]) * blend
            servo.angle = clamp(angle)
        time.sleep(dt)


def main():
    kit = ServoKit(channels=16)
    servos = {name: kit.servo[channel] for name, channel in SERVO_CHANNELS.items()}
    configure_servos(servos)

    print("Starting servo Mexican wave. Press Ctrl+C to stop.")
    try:
        sequence = list(WAVE_STEPS)
        current = {name: clamp(sequence[0][1][name]) for name in SERVO_CHANNELS}
        # Initialize servos at the first pose.
        for name, servo in servos.items():
            servo.angle = current[name]
        while True:
            for label, positions in sequence[1:] + sequence[:1]:
                target = {name: clamp(angle) for name, angle in positions.items()}
                angles = ", ".join(f"{name}={target[name]:>3.0f}" for name in ("A", "B", "C"))
                print(f"{label:<10} -> {angles}")
                interpolate_move(servos, current, target, TRANSITION_TIME, TRANSITION_STEPS)
                current = target
    except KeyboardInterrupt:
        print("\nStopping wave; leaving servos at last position.")


if __name__ == "__main__":
    main()
