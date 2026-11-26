# ESP32 Firmware: Learning and Modification Guide

How to understand the code in [`esp32/`](../esp32) and how to change it safely.
The full reference is [esp32-firmware.md](esp32-firmware.md) (architecture, wiring, calibration, extending).

The firmware is small: about 1,200 lines of C++ in 7 modules. You can understand all of it in an afternoon or two if you read it in the right order.

## What it is

It's the firmware for the rover's two drive motors. It takes `cmd_vel` from ROS, runs a 50 Hz loop that reads the encoders, runs PID and sets the motor PWM, and sends odometry back over micro-ROS. Two FreeRTOS tasks do all the work:

| Task | Core / priority | Job |
|---|---|---|
| `drivetrain` | core 0, prio 5 | Encoders → PID → PWM, and odometry |
| `micro_ros` | core 1, prio 3 | Agent connection, `cmd_vel` in, `odom`/`tf`/`wheel_status` out |

## Steps to fully understand it

### 1. Read the big picture first (about 30 min)

- [esp32-firmware.md](esp32-firmware.md) sections 1–3: how it fits into the rover, the file layout, and the threading model.
- `esp32/src/main.cpp` (55 lines): everything gets built and connected here.
- `esp32/include/config.h`: every pin, size, gain and rate. Values marked `TODO(measure)` are placeholders.

### 2. Read the code from the bottom layer up

For each file, read the `.h` first (the comments at the top explain the design), then the `.cpp`. Read along with the matching part of section 4 of the firmware doc.

| Order | File | What to learn |
|---|---|---|
| 1 | `src/hal/motor.*` | LEDC PWM, H-bridge wiring styles, what a duty from -1 to 1 means |
| 2 | `src/hal/encoder.*` | PCNT hardware counter, 4x quadrature decoding, how the 64-bit count handles overflow |
| 3 | `src/control/pid.*` | Feed-forward plus PID, anti-windup, derivative on measurement |
| 4 | `src/drivetrain/drivetrain.*` | **The core.** Turns (v, ω) into wheel speeds, limits acceleration, runs the control loop, computes odometry, uses a mutex |
| 5 | `src/ros/ros_module.h` | The plugin interface: `create` / `update` / `destroy` |
| 6 | `src/ros/ros_manager.*` | Connection state machine: wait for agent → connect → spin → reconnect |
| 7 | `src/modules/drivetrain_ros.*` | Connects ROS topics to `Drivetrain`'s thread-safe methods |

### 3. Learn the background topics you're missing

Only look these up when the code uses them:

- **FreeRTOS:** tasks, `xTaskCreatePinnedToCore`, mutexes (`SemaphoreHandle_t`), `vTaskDelayUntil`.
- **ESP32 peripherals:** LEDC (PWM) and PCNT (pulse counter).
- **micro-ROS / rclc:** node, publisher, subscription, executor, and the agent.
- **Diff-drive kinematics:** wheel speeds from (v, ω), and odometry integration.

### 4. Watch it run

Reading only gets you so far.

- Build and flash: `pio run`, then `pio run -t upload`. Stop the micro-ROS agent before flashing.
- Start the agent on the Pi. Then `ros2 topic echo /odom`, and send `cmd_vel` with `teleop_twist_keyboard` or `ros2 topic pub`.
- Run `ros2 topic echo /drivetrain/wheel_status`. It shows target, measured and duty for each wheel, so you can watch the PID work live.
- Run the calibration steps in section 9 of the firmware doc once: directions, counts per rev, wheel dimensions, `kff`, then PID. This is the best way to see how each part of the code behaves.

### 5. Test yourself

Try answering these by tracing through the code:

- What happens if `cmd_vel` stops arriving? (Hint: `cmdTimeoutMs`)
- What happens if the agent restarts while the rover is driving?
- Why does only the `micro_ros` task call micro-ROS functions?

## When you want to modify it

### First decide what kind of change it is

- **Tuning or wiring change** (pins, gains, wheel size, topic names, rates): edit only `include/config.h`. You shouldn't need to touch other code.
- **New feature** (IMU, shaker motor, battery monitor): follow the pattern in section 10 of the firmware doc.
  1. Write a hardware class in `src/hal/`, with its own task if it needs timing and thread-safe getters and setters.
  2. Write a `RosModule` in `src/modules/` that implements `create`, `destroy`, `update` and `executorHandles`.
  3. Add its config to `config.h`.
  4. Construct it and `addModule()` it in `main.cpp`.
- **More drive wheels (4WD/6WD):** add rows to `kWheels` in `config.h` (section 10 of the firmware doc).

### Rules the code depends on

Breaking these causes subtle bugs:

- **Only the micro-ROS task calls micro-ROS.** Hardware tasks share data only through mutex-protected getters and setters, the way `Drivetrain` does.
- **`destroy()` must be safe after a partial `create()`.** Use the `...Ok_` flags, as in `drivetrain_ros.cpp`.
- **Count your executor handles.** Every subscription, timer or service needs one in `executorHandles()`.
- **Respect resource limits:** 16 LEDC channels, 8 PCNT units, 8 modules, and the micro-ROS publisher/subscriber limits (section 10 of the firmware doc, "Resource limits").
- **Keep debug output off the serial port.** UART0 carries the micro-ROS stream, so `Serial.print` will corrupt it. Blink the LED or publish a debug topic instead.
- **Rebuild micro-ROS after changing its options.** If you change `board_microros_*` options or `extra_packages/`, run `pio run -t clean_microros` first.

### Workflow for each change

1. Commit (or back up) before you start, so you can always go back to a version that works.
2. Make one small change at a time, then build. Compile errors are cheap, so catch them early.
3. Flash and test with the wheels **off the ground** first.
4. Check the safety behaviour still works: stopping on timeout and stopping when the agent disconnects (section 7 of the firmware doc).
5. Update `config.h` comments and the doc if you changed behaviour.
