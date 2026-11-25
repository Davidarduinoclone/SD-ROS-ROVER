# ESP32 drivetrain firmware

micro-ROS + FreeRTOS firmware for the rover's two-motor drivetrain (encoders, PID, PWM, odometry).

```bash
pio run              # build
pio run -t upload    # flash (stop the micro-ROS agent first)
```

All settings are in [`include/config.h`](include/config.h).

Full documentation (architecture, wiring, ROS topics, calibration, PID tuning, adding an IMU or more motors):
**[../docs/esp32-firmware.md](../docs/esp32-firmware.md)**
