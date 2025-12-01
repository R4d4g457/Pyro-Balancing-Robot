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
- Py: `sudo raspi-config` --
Interface Options → I2C → Enable, then reboot), or edit `/boot/config.txt` and ensure `dtparam=i2c_arm=on` is present. (otherwise you get `Make sure I2C is enabled` errors)
- Py: `sudo apt install -y i2c-tools` for `i2cdetect`


## Auto-start on boot (pyro service)

- Copy `scripts/start_pyro.sh` and `scripts/pyro.service` to the Pi with the rest of the repo.
- Edit the paths inside both files if your user, repo directory, or virtual environment differs from the defaults (`/home/pyro/pyro` and `/home/pyro/venv`).
- On the Pi:
  - `sudo cp scripts/pyro.service /etc/systemd/system/pyro.service`
  - `sudo systemctl daemon-reload`
  - `sudo systemctl enable pyro.service`
  - `sudo systemctl start pyro.service` (or reboot)
  - Tail logs with `journalctl -u pyro.service -f`

`start_pyro.sh` activates `~/venv`, changes into the repo directory, and runs `python3 main.py`. The systemd unit ensures it starts at boot and restarts on failure.


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
