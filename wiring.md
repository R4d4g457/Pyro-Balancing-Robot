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

### Updating code & restarting the service

1. Copy new files (or `git pull`) into `/home/pyro/pyro`.
2. Run `scripts/restart_pyro.sh` from that directory (it calls `sudo systemctl daemon-reload && sudo systemctl restart pyro.service` and prints the status). You can also run the commands manually if you prefer.
3. Tail logs with `sudo journalctl -u pyro.service -f` to confirm the controller is healthy.

> `scripts/restart_pyro.sh` needs sudo privileges; run it from the Pi or via SSH as the `pyro` user.


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


# Calibration checklist

## Servo travel limits

The `clamp()` helper inside `controller.py` (lines 13–15) constrains every command to 19–90°. Adjust these bounds if your hardware safely supports a wider range. To verify the extremes:

1. Stop the pyro service (`sudo systemctl stop pyro.service`).
2. Run a quick manual sweep:
   ```bash
   python3 - <<'PY'
   from controller import RobotController
   import time
   rc = RobotController()
   for angle in (20, 30, 40, 60, 80, 90):
       print("Moving to", angle)
       rc.set_motor_angles(angle, angle, angle)
       time.sleep(2)
   PY
   ```
3. Update the clamp range if the servos can move farther, or tighten it if they bind.

## Servo offsets / level plate

Use the offsets in `RobotController.set_motor_angles` (lines 54–60) to level the top plate:

1. Temporarily edit the offsets tuple (e.g. `servo_offsets=(+2.0, -1.5, 0.0)`).
2. Restart the service (or re-run the manual script) and observe the plate. Repeat until the top plate is level and the ball stays put when idle.

## IMU alignment check

1. Run the controller with debug logs: `TILT_DEBUG=1 PYRO_AXIS_ROT_DEG=0 python3 main.py` (stop the service first).
2. Gently pitch the platform forward/back and observe the printed `raw` vs `rot` values.
3. Adjust environment variables and rerun until the axes line up:
   - `PYRO_AXIS_ROT_DEG=…` rotates the IMU frame around the vertical axis (positive values rotate clockwise when looking down). Start with ±30° increments until the response aligns with the platform.
   - `PYRO_INVERT_PITCH=1` or `PYRO_INVERT_ROLL=1` flips individual axes if they run backwards.
   - `PYRO_PITCH_GAIN=…` / `PYRO_ROLL_GAIN=…` scale how aggressively each axis feeds into the PID (use values >1.0 for more correction if the motion feels weak, <1.0 if it overshoots).
   - `PYRO_PID_KP`, `PYRO_PID_KI`, `PYRO_PID_KD` change the PID controller gains; `PYRO_PID_MAX_OUT` raises/lowers the saturation limit (degrees) before commands feed the kinematics.
   - `PYRO_OUTPUT_GAIN` multiplies the PID outputs before converting to servo angles if you need stronger motion without altering PID dynamics.
4. Once satisfied, add the chosen variables to `scripts/pyro.service` as extra `Environment=` lines so the service uses the same calibration (e.g. `Environment=PYRO_AXIS_ROT_DEG=15`, `Environment=PYRO_OUTPUT_GAIN=1.3`).
5. Re-enable the service with `scripts/restart_pyro.sh`.

Document your final clamp limits and offsets in `controller.py` (with comments) so future rebuilds use the calibrated values.
