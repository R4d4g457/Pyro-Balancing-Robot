# RaspberryPi setup:

- Flash MicroSD card with `Raspberry Pi OS (Legacy, 32-bit) Lite` 
“A port of Debian Bookworm with security updates and no desktop environment”
- Add `wpa_supplicant.conf` with WiFi network details via MicroSD card
- Confirm `$ ssh pyro@pyro.local` works
- `scp requirements.txt pyro@pyro.local:`
- Pi: `sudo apt-get install pip`
- Pi: `python3 -m venv venv`
- Pi: `. venv/bin/activate`
- Pi: `pip install -r requirements.txt`
- Pi: `sudo raspi-config` --
Interface Options → I2C → Enable, then reboot), or edit `/boot/config.txt` and ensure `dtparam=i2c_arm=on` is present. (otherwise you get `Make sure I2C is enabled` errors)
- Pi: `sudo apt install -y i2c-tools` for `i2cdetect`

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
3. If you still need to bias or smooth the IMU-derived commands, use the complementary-filter alphas (`PITCH_ALPHA`, `ROLL_ALPHA`), the PID output EMA (`OUTPUT_EMA_ALPHA`), and/or small output deadband (`OUTPUT_DEADBAND`) rather than neutral blending.
4. Add the chosen values to `scripts/pyro.service` so they persist across reboots.

## IMU alignment check

1. Run the controller with debug logs: `DEBUG=1 python3 main.py` (stop the service first).
2. Gently pitch the platform forward/back and observe the printed `raw` vs `rot` values.
3. Adjust environment variables and rerun until the axes line up:
   - `AXIS_ROT_DEG=…` rotates the IMU frame around the vertical axis (positive values rotate clockwise when looking down). Start with ±30° increments until the response aligns with the platform (or provide calibration vectors).
   - `INVERT_PITCH=1` or `INVERT_ROLL=1` flips individual axes if they run backwards.
   - `PITCH_OFFSET` / `ROLL_OFFSET` remove static bias (e.g. if level ground reads `+0/-4.5`, set the offsets to those numbers so the transformed values centre on zero).
   - `KP`, `KI`, `KD`, `MAX_TILT_DEG` adjust the PID response and tilt clamp.
   - `PITCH_ALPHA`, `ROLL_ALPHA` adjust the complementary filter weighting per axis; `OUTPUT_EMA_ALPHA` applies a low-pass filter to the PID outputs and `OUTPUT_DEADBAND` clips tiny commands to zero before commanding servos.
4. Once satisfied, add the chosen variables to `scripts/pyro.service` as extra `Environment=` lines so the service uses the same calibration (e.g. `Environment=PITCH_ALPHA=0.65`).
5. Re-enable the service with `scripts/restart_pyro.sh`.

Document your final clamp limits and offsets in `controller.py` (with comments) so future rebuilds use the calibrated values.
