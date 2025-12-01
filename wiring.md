# RaspberryPi setup:

- Flash MicroSD card with `Raspberry Pi OS (Legacy, 32-bit) Lite` 
“A port of Debian Bookworm with security updates and no desktop environment”
- Add `wpa_supplicant.conf` with WiFi network details via MicroSD card
- Confirm `$ ssh pyro@pyro.local` works
- `scp requirements.txt pyro@pyro.local:`
- Py: `sudo apt-get install pip`
- Py: `python3 -m venv venv`
- Py: `. venv/bin/activate`
- Py: `pip install -r requirements.txt`
- Py `sudo raspi-config` --
Interface Options → I2C → Enable, then reboot), or edit `/boot/config.txt` and ensure `dtparam=i2c_arm=on` is present. (otherwise you get `Make sure I2C is enabled` errors)


# Files to copy to RaspberryPi:

- `main.py`
- `controller.py`
- `imu.py`
- `robotKinematics.py`
- `requirements.txt` (as above)


# Expected wiring

## I²C bus (shared by PCA9685 + MPU6050)
- Pi SDA1 (GPIO2, physical pin 3) → PCA9685 SDA and MPU6050 SDA. The IMU driver in `imu.py` (lines 8–11) opens bus 1 (pins 3/5); `ServoKit` in `controller.py` (lines 21–47) uses the same bus.
- Pi SCL1 (GPIO3, physical pin 5) → PCA9685 SCL and MPU6050 SCL.
- Leave the MPU6050 AD0 low (address `0x68`) and the PCA9685 address pins floating/low (default `0x40`) so both devices work without code changes.

## Power rails
- Common ground: connect Pi GND to PCA9685 GND, MPU6050 GND, and the external servo supply ground (controller.py assumes servos reference Pi ground).
- Logic supply:
    - Pi 3V3 → MPU6050 VCC (3.3 V sensor).
    - Pi 5V → PCA9685 VCC (most breakouts expect 5 V logic).
- Servo power: feed the PCA9685 breakout V+ with your high‑current servo supply; ensure that supply ground is tied back to Pi GND.

## Servos
- Plug servo signal wires into PCA9685 channels 13, 15, 14 respectively (matches `servo_channels = (13, 15, 14)` in `controller.py` lines 23–41).
- Servo power and ground stay on the PCA9685 breakout (V+/GND).

Notes
- With these connections the current code can initialize `ServoKit`, read the MPU6050 on I2C bus 1, and drive the three servos without further pin configuration.
