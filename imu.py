import math
import time

import smbus2


class MPU6050:
    def __init__(
        self,
        bus_id=1,
        address=0x68,
        alpha_pitch=0.98,
        alpha_roll=0.98,
        gyro_calibration_samples=500,
    ):
        self.bus = smbus2.SMBus(bus_id)
        self.address = address

        # Wake up device
        self.bus.write_byte_data(self.address, 0x6B, 0)

        self.accel_scale = 16384.0
        self.gyro_scale = 131.0

        self.alpha_pitch = alpha_pitch
        self.alpha_roll = alpha_roll
        self.pitch = 0.0
        self.roll = 0.0
        self.last_time = time.time()
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self._calibrate_gyro(gyro_calibration_samples)

    def _calibrate_gyro(self, samples):
        accum_x = accum_y = 0.0
        for _ in range(samples):
            accum_x += self._read_word(0x43) / self.gyro_scale
            accum_y += self._read_word(0x45) / self.gyro_scale
            time.sleep(0.002)
        self.gyro_bias_x = accum_x / samples
        self.gyro_bias_y = accum_y / samples

    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) | low
        if val & 0x8000:
            val = -((65535 - val) + 1)
        return val

    def read(self):
        ax = self._read_word(0x3B) / self.accel_scale
        ay = self._read_word(0x3D) / self.accel_scale
        az = self._read_word(0x3F) / self.accel_scale

        gx = self._read_word(0x43) / self.gyro_scale - self.gyro_bias_x
        gy = self._read_word(0x45) / self.gyro_scale - self.gyro_bias_y

        t = time.time()
        dt = max(1e-3, t - self.last_time)
        self.last_time = t

        pitch_acc = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
        roll_acc = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))

        self.pitch += gy * dt
        self.roll += gx * dt

        self.pitch = self.alpha_pitch * self.pitch + (1.0 - self.alpha_pitch) * pitch_acc
        self.roll = self.alpha_roll * self.roll + (1.0 - self.alpha_roll) * roll_acc

        return self.pitch, self.roll
