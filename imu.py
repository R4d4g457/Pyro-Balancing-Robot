import math
import time

try:
    import board
    import busio
    from adafruit_mpu6050 import MPU6050 as AdafruitMPUSensor

    _HAVE_ADAFRUIT = True
except ImportError:
    _HAVE_ADAFRUIT = False

import smbus2


class _AdafruitIMU:
    def __init__(self, alpha=0.97, gyro_calibration_samples=500, i2c_bus=None, **_):
        if not _HAVE_ADAFRUIT:
            raise RuntimeError("Adafruit MPU6050 driver unavailable")

        self.i2c = i2c_bus or busio.I2C(board.SCL, board.SDA)
        self.sensor = AdafruitMPUSensor(self.i2c)

        self.alpha = alpha
        self.pitch = 0.0
        self.roll = 0.0
        self.last_time = time.time()
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self._calibrate_gyro(gyro_calibration_samples)

    def _calibrate_gyro(self, samples):
        accum_x = accum_y = 0.0
        for _ in range(samples):
            gx, gy, _ = self.sensor.gyro
            accum_x += math.radians(gx)
            accum_y += math.radians(gy)
            time.sleep(0.002)
        self.gyro_bias_x = accum_x / samples
        self.gyro_bias_y = accum_y / samples

    def read(self):
        accel = self.sensor.acceleration
        gyro = self.sensor.gyro

        gx = math.radians(gyro[0]) - self.gyro_bias_x
        gy = math.radians(gyro[1]) - self.gyro_bias_y

        t = time.time()
        dt = max(1e-3, t - self.last_time)
        self.last_time = t

        pitch_acc = math.degrees(math.atan2(accel[0], math.sqrt(accel[1] ** 2 + accel[2] ** 2)))
        roll_acc = math.degrees(math.atan2(accel[1], math.sqrt(accel[0] ** 2 + accel[2] ** 2)))

        self.pitch += math.degrees(gy * dt)
        self.roll += math.degrees(gx * dt)

        self.pitch = self.alpha * self.pitch + (1.0 - self.alpha) * pitch_acc
        self.roll = self.alpha * self.roll + (1.0 - self.alpha) * roll_acc

        return self.pitch, self.roll


class _SMBusIMU:
    def __init__(self, alpha=0.95, bus_id=1, address=0x68, gyro_calibration_samples=500, **_):
        self.bus = smbus2.SMBus(bus_id)
        self.address = address
        self.bus.write_byte_data(self.address, 0x6B, 0)

        self.accel_scale = 16384.0
        self.gyro_scale = 131.0
        self.alpha = alpha

        self.pitch = 0.0
        self.roll = 0.0
        self.last_time = time.time()

        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self._calibrate_gyro(gyro_calibration_samples)

    def _calibrate_gyro(self, samples):
        accum_x = accum_y = 0.0
        for _ in range(samples):
            gx = self._read_word(0x43) / self.gyro_scale
            gy = self._read_word(0x45) / self.gyro_scale
            accum_x += gx
            accum_y += gy
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

        self.pitch = self.alpha * self.pitch + (1.0 - self.alpha) * pitch_acc
        self.roll = self.alpha * self.roll + (1.0 - self.alpha) * roll_acc

        return self.pitch, self.roll


class MPU6050:
    def __init__(self, **kwargs):
        self._impl = None

        if _HAVE_ADAFRUIT:
            try:
                self._impl = _AdafruitIMU(**kwargs)
            except RuntimeError:
                pass

        if self._impl is None:
            self._impl = _SMBusIMU(**kwargs)

    def read(self):
        return self._impl.read()
