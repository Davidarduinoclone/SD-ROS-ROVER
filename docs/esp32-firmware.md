# ESP32 Drivetrain Firmware

This document explains the PlatformIO project in [`../esp32`](../esp32). The firmware turns an ESP32 into the rover's drivetrain controller. It receives velocity commands from ROS 2 through micro-ROS, drives two DC motors with PWM, closes a PID speed loop on each wheel using its encoder, and publishes wheel odometry back to ROS 2.

The code is laid out so more hardware (an IMU, more drive motors, the shaker motor, a battery monitor...) can be added without changing the existing parts. Section 10 walks through how.

---

## 1. How it fits into the rover

```
 Raspberry Pi (Docker container)                         ESP32
┌────────────────────────────────┐   USB serial   ┌─────────────────────────────┐
│ Nav2 / teleop  ── /cmd_vel ──► │                │                             │
│                                │ micro-ROS agent│  micro_ros task             │
│ slam_toolbox ◄── /odom, /tf ── │ ◄────────────► │    cmd_vel ─► Drivetrain    │
│ rqt_plot  ◄── wheel_status ─── │   115200 baud  │    Drivetrain ─► odom/tf    │
└────────────────────────────────┘                │                             │
                                                  │  drivetrain task (50 Hz)    │
                                                  │    encoders ─► PID ─► PWM   │
                                                  └──────┬──────────────┬───────┘
                                                         │              │
                                                   motor driver    encoders
                                                   (H-bridge)      (quadrature)
```

The ESP32 runs **micro-ROS**, a small ROS 2 client for microcontrollers. It can't join the ROS 2 network directly. Instead it talks to the **micro-ROS agent**, a program in the rover's Docker container (the `agent_serial` alias) that relays everything between the ESP32 and the normal ROS 2 network. To every other ROS 2 node, the ESP32 looks like an ordinary node called `esp32_drivetrain`.

---

## 2. Project layout

```
esp32/
├── platformio.ini            build settings, micro-ROS distro + transport
├── include/
│   └── config.h              ALL pins, dimensions, gains, rates, topic names
├── extra_packages/           extra ROS message packages compiled into micro-ROS
│   └── tf2_msgs/             minimal tf2_msgs (TFMessage only), needed for /tf
└── src/
    ├── main.cpp              creates the objects, registers modules, starts tasks
    ├── hal/                  hardware drivers (no ROS, no control logic)
    │   ├── motor.h/.cpp      HBridgeMotor: LEDC PWM + direction pins
    │   └── encoder.h/.cpp    QuadratureEncoder: PCNT hardware counter
    ├── control/
    │   └── pid.h/.cpp        Pid: velocity PID with feed-forward + anti-windup
    ├── drivetrain/
    │   └── drivetrain.h/.cpp Drivetrain: kinematics, per-wheel PID, odometry, control task
    ├── ros/                  micro-ROS plumbing (no hardware)
    │   ├── ros_module.h      RosModule interface every ROS feature implements
    │   └── ros_manager.h/.cpp RosManager: agent connection, node, executor, ROS task
    └── modules/              glue between a subsystem and ROS
        └── drivetrain_ros.h/.cpp  cmd_vel subscriber, odom/tf/wheel_status publishers
```

Each layer only depends on the layers below it:

| Layer          | Knows about            | Does not know about        |
|----------------|------------------------|----------------------------|
| `hal/`         | ESP32 hardware         | PID, kinematics, ROS        |
| `control/`     | maths only             | hardware, ROS               |
| `drivetrain/`  | `hal/`, `control/`     | ROS                         |
| `ros/`         | micro-ROS              | any specific hardware       |
| `modules/`     | one subsystem + `ros/` | other subsystems            |
| `config.h`     | everything (it only holds values) |                  |
| `main.cpp`     | everything (it wires the pieces together) |          |

This separation is what makes the project easy to extend. For example, the drivetrain can be tested without ROS, and a new sensor never needs changes to the drivetrain.

---

## 3. Threading model (FreeRTOS)

The ESP32 has two CPU cores and runs FreeRTOS. The firmware uses two tasks:

| Task         | Core | Priority | Rate             | Job                                                     |
|--------------|------|----------|------------------|---------------------------------------------------------|
| `drivetrain` | 0    | 5        | 50 Hz (fixed)    | Read encoders, run PID, set PWM, integrate odometry      |
| `micro_ros`  | 1    | 3        | as fast as possible | Agent connection, receive `cmd_vel`, publish topics   |

The Arduino `loop()` deletes itself; everything happens in these tasks.

**Why two tasks?** The control loop must run at a steady rate. A serial hiccup or a slow agent must never delay the PID, otherwise the wheels jerk. The control task has higher priority and its own core, and uses `vTaskDelayUntil` for an exact period. The `micro_ros` task can block on serial without affecting motor control.

**Rules that keep it safe:**

1. **Only the `micro_ros` task calls micro-ROS functions.** micro-ROS is not thread-safe. Modules publish from their `update()` hook, which runs in that task.
2. **Shared data goes through thread-safe methods.** `Drivetrain::setCommand()`, `odometry()` and `wheelStatus()` take a FreeRTOS mutex. The lock is held only to copy a few numbers, never during I/O.
3. **The control task never waits on ROS.** If commands stop arriving, it just uses the last values until the timeout (section 7).

---

## 4. How each part works

### 4.1 Motor driver (`hal/motor`)

`HBridgeMotor` drives one DC motor through an H-bridge. Speed is PWM on an LEDC channel (the ESP32 PWM peripheral) at 20 kHz, which is above human hearing, so the motors don't whine, with 10-bit resolution (0–1023).

`setDuty(d)` takes a value from −1 (full reverse) to +1 (full forward). Two wirings are supported:

| Driver type                    | Pins used             | Config                                 |
|--------------------------------|-----------------------|----------------------------------------|
| IN1/IN2 + PWM (TB6612FNG, L298N, DRV8871 via EN) | `in1`, `in2`, `pwm` | `{pwm, in1, in2, channel, inverted}` |
| DIR + PWM (Cytron MD10C/MD13S) | `in1` = DIR, `pwm`    | `{pwm, dir, -1, channel, inverted}`    |

With duty 0 both inputs go low, so the motor **coasts** rather than brakes.

`inverted` flips the direction. Motors on the right side are mounted mirrored, so they usually need it set to make positive duty drive the rover **forward**.

### 4.2 Encoders (`hal/encoder`)

`QuadratureEncoder` uses the ESP32 **PCNT** (pulse counter) peripheral, which counts pulses in hardware. No interrupt fires per pulse, so even fast encoders cost no CPU time.

- **4× decoding:** every rising and falling edge of both A and B is counted. For example, an 11 PPR motor encoder gives 44 counts per motor revolution, and with a 30:1 gearbox that is 1320 counts per wheel revolution.
- **Direction** comes from the A/B phase relationship. `inverted` flips the sign.
- **Overflow:** the hardware counter is 16-bit. When it reaches ±30000 it resets and raises an interrupt, and the interrupt adds ±30000 to a 64-bit total. `count()` returns that total, so it effectively never wraps.
- **Glitch filter:** pulses shorter than 100 APB cycles (1.25 µs) are ignored. This rejects electrical noise from the motors.
- The ESP32 has 8 PCNT units, so up to 8 encoders.

### 4.3 PID controller (`control/pid`)

Each wheel has its own speed controller. Its input is the target and measured wheel speed in rad/s, and its output is PWM duty:

```
duty = kff·target  +  kp·error  +  ki·∫error dt  −  kd·d(measured)/dt
```

| Term              | Role                                                                                         |
|-------------------|----------------------------------------------------------------------------------------------|
| Feed-forward `kff`| The "open-loop guess": duty needed for a given speed. Does most of the work, so the response is fast |
| Proportional `kp` | Corrects the remaining error immediately                                                     |
| Integral `ki`     | Removes steady error (load, slope, battery sag, friction)                                     |
| Derivative `kd`   | Damps oscillation. Usually 0 for wheel speed, since encoder speed is noisy                   |

Details:

- **Derivative on measurement** rather than on error, so a sudden setpoint change doesn't produce a spike ("derivative kick").
- **Anti-windup:** when the output is saturated at ±1, the integral stops growing in the saturating direction. Without this, after a stall the integral would be huge and the wheel would overshoot badly.
- **Reset while parked:** when the target is 0 and the wheel is stopped, the PID is reset and duty is 0. That stops the integral from slowly creeping and the motor from humming at rest.

### 4.4 Drivetrain (`drivetrain/drivetrain`)

Every 20 ms (50 Hz) the control task runs `step()`:

1. **Read the command** `(v, ω)`. If it is older than `cmdTimeoutMs` (500 ms), use `(0, 0)`.
2. **Inverse kinematics** turns body velocity into wheel speeds (rad/s), with r = wheel radius and L = track width:
   ```
   ω_left  = (v − ω·L/2) / r
   ω_right = (v + ω·L/2) / r
   ```
3. **Saturation:** if either wheel would exceed `maxWheelSpeed`, both are scaled down by the same factor. This keeps the turn radius, so the rover slows down along the requested arc instead of going straighter than asked.
4. **Acceleration limit:** each wheel's target moves toward the desired speed by at most `maxWheelAccel·dt`. This avoids wheel slip, current spikes and tipping over.
5. **Per wheel:** read the encoder → speed = Δcounts × 2π / countsPerRev / dt → low-pass filter (`velocityFilterAlpha`) → PID → `setDuty()`.
6. **Odometry:** the average distance travelled by each side is integrated with the midpoint method:
   ```
   ds = (d_left + d_right) / 2
   dθ = (d_right − d_left) / L
   x += ds·cos(θ + dθ/2)
   y += ds·sin(θ + dθ/2)
   θ += dθ
   ```

**Skid steer:** each wheel is tagged `Side::Left` or `Side::Right`. With 4 or 6 wheels, every wheel on a side gets the same target and runs its own PID, and odometry averages the side. Going from 2WD to 4WD only means adding rows in `config.h`.

### 4.5 micro-ROS connection (`ros/ros_manager`)

`RosManager` runs a state machine so the ESP32 and the agent can be started in any order, and the agent can be restarted at any time:

```
            ping OK                     create OK
 WAITING ───────────► AGENT_AVAILABLE ───────────► CONNECTED ──┐
 AGENT    ◄─────────── (create failed)              │  ▲        │ spin executor,
   ▲                                     ping fails │  └────────┘ module update()
   └──────────────────── DISCONNECTED ◄─────────────┘
                         (destroy everything)
```

- **WAITING_AGENT:** pings the agent every 500 ms. The status LED blinks.
- **AGENT_AVAILABLE:** creates the node, the executor and every module's publishers and subscriptions, then synchronises the clock with the agent so message timestamps match the Pi's time.
- **CONNECTED:** the status LED is solid. Each loop it processes incoming messages (`rclc_executor_spin_some`) and calls each module's `update()`. It pings the agent every second and re-syncs the clock every 60 s.
- **DISCONNECTED:** destroys everything and goes back to waiting.

When the agent disappears, `cmd_vel` stops arriving, so the drivetrain's command timeout stops the motors.

The executor size is calculated automatically from each module's `executorHandles()`.

### 4.6 Drivetrain ROS module (`modules/drivetrain_ros`)

Connects the `Drivetrain` to ROS:

- `cmd_vel` callback → `drivetrain.setCommand(linear.x, angular.z)`
- `update()` publishes `odom` and `/tf` at 20 Hz and `drivetrain/wheel_status` at 10 Hz.

Message buffers are allocated once, and frame names point to constant strings, so nothing is allocated while running.

---

## 5. ROS interface

All topics without a leading `/` are placed under the node namespace (`config::kRos`, empty by default).

| Topic                      | Type                          | Direction | Rate   | Content                                                     |
|----------------------------|-------------------------------|-----------|--------|-------------------------------------------------------------|
| `cmd_vel`                  | `geometry_msgs/Twist`         | in        | –      | `linear.x` m/s (forward +), `angular.z` rad/s (left/CCW +)  |
| `odom`                     | `nav_msgs/Odometry`           | out       | 20 Hz  | Pose (x, y, yaw) and body velocity in the `odom` frame       |
| `/tf`                      | `tf2_msgs/TFMessage`          | out       | 20 Hz  | `odom → base_link` (if `publishTf` is true)                 |
| `drivetrain/wheel_status`  | `std_msgs/Float32MultiArray`  | out       | 10 Hz  | Per wheel: `[target rad/s, measured rad/s, duty]`            |

Node name: `esp32_drivetrain`.

The other fields of `cmd_vel` (linear.y, angular.x...) are ignored. A differential drive can't move sideways.

**Covariance.** Odometry reports small variance for x, y and yaw and very large variance (1e6) for z, roll and pitch, meaning "not measured". This is what `robot_localization` expects for a ground robot.

**TF.** For SLAM or Nav2, something must publish `odom → base_link`. By default the ESP32 does it. When you later fuse wheel odometry with an IMU using `robot_localization`, the EKF publishes that transform instead. Then set `publishTf` to `false`, otherwise two publishers fight over the same transform.

---

## 6. Wiring

### Default pins (`include/config.h`)

| Signal              | Left wheel | Right wheel |
|---------------------|-----------:|------------:|
| Motor PWM           | GPIO 25    | GPIO 33     |
| Motor IN1 / DIR     | GPIO 26    | GPIO 32     |
| Motor IN2           | GPIO 27    | GPIO 4      |
| Encoder A           | GPIO 34    | GPIO 36 (VP)|
| Encoder B           | GPIO 35    | GPIO 39 (VN)|
| Status LED          | GPIO 2 (on-board) |      |
| micro-ROS serial    | USB (UART0, GPIO 1/3) |  |

These are a starting point. Change them in `config.h` to match your wiring.

### Pin rules for ESP32-WROOM boards

- **Never use GPIO 6–11.** They are connected to the flash chip.
- **Don't use GPIO 1 and 3.** That is UART0, which carries the micro-ROS connection over USB.
- **Avoid strapping pins 0, 2, 5, 12, 15 for motors.** Their state at boot selects boot modes, and a motor driver pulling on them can stop the board from booting or flashing. GPIO 12 is the worst: if it is high at boot, the flash voltage is wrong.
- **GPIO 34–39 are input only and have no internal pull-ups.** They suit encoders. Hall-effect encoders with open-collector outputs need external pull-up resistors (4.7 kΩ to 3.3 V). Encoders with push-pull outputs don't.
- **ESP32 inputs are 3.3 V.** If the encoder is powered from 5 V and outputs 5 V, use a level shifter or a resistor divider.
- Every motor needs a **unique LEDC channel** (0–15).

### Power

- Power the motors from the battery through the driver. **Never from the ESP32's 3.3 V or 5 V pin.**
- **Connect the grounds**: ESP32 GND, driver logic GND, encoder GND and battery negative.
- Put a 100 nF capacitor across each motor's terminals, and a bulk capacitor (470 µF+) on the driver's motor supply. Motor noise is the most common cause of encoder glitches and ESP32 resets.

---

## 7. Safety behaviour

| Situation                              | Result                                                                |
|----------------------------------------|-----------------------------------------------------------------------|
| No `cmd_vel` for 500 ms                | Targets ramp to 0, motors stop                                        |
| Agent stops / USB unplugged / Pi crash | Same: no commands → timeout → stop. ESP32 then waits for the agent    |
| ESP32 boot / reset                     | Motors initialised to duty 0 before anything else runs                |
| Command faster than the wheels allow   | Both wheels scaled down together (turn radius preserved)               |
| Sudden large command                   | Limited by `maxWheelAccel`                                            |
| Hardware init fails (bad config)       | Firmware halts with motors off; status LED blinks fast (10 Hz)        |

Teleop nodes publish continuously only while a key or the joystick is held. With `teleop_twist_keyboard`, the rover stops 0.5 s after the last key press, which is expected.

---

## 8. Building, flashing and running

### Build and flash

Use the PlatformIO extension in VS Code / code-server (open the `esp32` folder), or the command line:

```bash
cd esp32
pio run                 # build
pio run -t upload       # flash (ESP32 on USB)
```

The **first build takes several minutes.** PlatformIO downloads micro-ROS and compiles it for the ESP32, together with the packages in `extra_packages/`. Later builds are fast. If you change `board_microros_distro` or `board_microros_transport` in `platformio.ini`, or anything in `extra_packages/`, run `pio run -t clean_microros` once.

The micro-ROS build needs `git` and an internet connection, and it uses Python packages that it installs into PlatformIO's own environment. The PlatformIO VS Code extension sets that environment up automatically. A firmware build currently uses about 14% of RAM and 29% of flash, which leaves plenty of room for more modules.

**No serial monitor or `Serial.print`.** The USB serial port carries the micro-ROS stream, and any text printed to it corrupts that stream. Use the status LED, or publish debug values on a topic. (`CORE_DEBUG_LEVEL=0` in `platformio.ini` keeps the Arduino core's own log messages off the port for the same reason.)

The agent and the flasher can't use the port at the same time. **Stop the agent before uploading.**

### Run

1. Plug the ESP32 into the Pi. With the udev rules from [docker.md](docker.md#6-udev-rules-99-roverrules) it appears as `/dev/esp32`.
2. In the rover container:
   ```bash
   agent_serial          # = micro_ros_agent serial --dev /dev/esp32 -b 115200
   ```
   The status LED changes from blinking to solid when the ESP32 connects. If it stays blinking, press the ESP32's EN (reset) button.
3. In another terminal:
   ```bash
   ros2 node list                          # /esp32_drivetrain
   ros2 topic list                         # /cmd_vel /odom /tf /drivetrain/wheel_status
   ros2 topic echo /odom
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

**Put the rover on a stand, with its wheels off the ground, for the first test.**

### Wi-Fi instead of USB

1. In `platformio.ini`: `board_microros_transport = wifi`
2. In `main.cpp`: replace `set_microros_serial_transports(Serial)` with the commented `set_microros_wifi_transports(...)` line, filling in the SSID, password and Pi IP address.
3. `pio run -t clean_microros`, then build and flash.
4. On the Pi: `agent_udp` (port 8888).

With Wi-Fi, `Serial.print` debugging works again.

---

## 9. Calibration and tuning

Do these in order, with the rover on a stand. Everything is in `include/config.h`.

### Step 1: Directions

Drive forward slowly (`i` in teleop_twist_keyboard), then `ros2 topic echo /drivetrain/wheel_status`. The data is `[L_target, L_measured, L_duty, R_target, R_measured, R_duty]`.

| What you see                                                        | Fix                                      |
|---------------------------------------------------------------------|------------------------------------------|
| A wheel spins backwards                                              | Toggle that wheel's **motor** `inverted` |
| Wheel spins forward but its `measured` is negative                   | Toggle that wheel's **encoder** `inverted` |
| Wheel runs away at full speed and never settles                      | Encoder sign is wrong (positive duty → negative speed → PID pushes harder). Toggle encoder `inverted` |
| `measured` stays 0                                                   | Encoder wiring, pull-ups, or encoder power |

A correct wheel spins forward and shows a positive `measured` that follows `target`.

### Step 2: Counts per revolution

`countsPerRev` = 4 × encoder PPR × gear ratio. Use the datasheet, or measure it:

1. Mark a wheel, then turn it exactly 10 revolutions by hand (firmware running, no commands).
2. Compare the odometry change with the expected distance 10 × 2π × r, and correct `countsPerRev` by the ratio.

### Step 3: Wheel radius and track width

- **Wheel radius:** drive 2 m straight on the floor and compare `/odom` x with a tape measure. `new_radius = old_radius × real / reported`.
- **Track width:** rotate in place 5 full turns (10π rad) and compare the reported yaw change. `new_track = old_track × reported / real`. On a skid-steer rover the effective track width is usually **larger** than the measured one, because the wheels slip sideways when turning. Always calibrate it this way.

### Step 4: Feed-forward (`kff`)

1. Temporarily set `kp = ki = kd = 0` and `kff = 1.0 / maxWheelSpeed`.
2. Command a medium speed and read `measured` and `duty` in `wheel_status`.
3. Set `kff ≈ duty / measured`. The wheel should now get close to the target speed on its own.

Also set `maxWheelSpeed` a bit below the no-load speed at full duty, so the PID always has headroom.

### Step 5: PID

Plot target against measured:

```bash
ros2 run rqt_plot rqt_plot /drivetrain/wheel_status/data[0] /drivetrain/wheel_status/data[1]
```

1. Raise `kp` until the response is quick but not oscillating, then back off ~30%.
2. Raise `ki` until the steady error disappears within ~0.5 s without overshoot.
3. Leave `kd = 0` unless it oscillates and more filtering doesn't help.
4. Repeat on the floor, where the load is higher than on a stand.

If `measured` is very jumpy at low speed, the encoder has too few counts per control period. Lower `velocityFilterAlpha` (more filtering, e.g. 0.3) or increase `controlPeriodMs`.

---

## 10. Extending the firmware

### Pattern

Every new feature has up to three parts:

1. **Hardware class** (`src/hal/` or its own folder): talks to the device. If it needs regular timing, it runs its own FreeRTOS task and exposes thread-safe getters and setters, like `Drivetrain` does.
2. **ROS module** (`src/modules/`): a class implementing `RosModule` that creates its publishers and subscriptions in `create()`, removes them in `destroy()`, and publishes in `update()`.
3. **Registration** in `main.cpp`: construct both, `begin()` the hardware, `rosManager.addModule(&module)`.

Settings go into `config.h`.

### Example: adding an IMU

Here is how a `sensor_msgs/Imu` publisher would look, for example for a BNO055 or MPU6050 on I2C (SDA 21, SCL 22).

**`src/hal/imu.h`**: hardware side, own task, thread-safe reading

```cpp
#pragma once
#include <Arduino.h>
#include <freertos/semphr.h>

struct ImuReading {
  float ax, ay, az;      // m/s^2
  float gx, gy, gz;      // rad/s
  float qx, qy, qz, qw;  // orientation (if the sensor fuses it)
};

class Imu {
 public:
  bool begin(uint32_t periodMs);          // init I2C + sensor, start task
  ImuReading latest() const;              // mutex-protected copy
 private:
  static void taskEntry(void* arg);       // loop: read sensor, store under mutex
  mutable SemaphoreHandle_t mutex_ = nullptr;
  ImuReading reading_{};
  uint32_t periodMs_ = 10;
};
```

**`src/modules/imu_ros.h`**: ROS side

```cpp
#pragma once
#include <sensor_msgs/msg/imu.h>
#include "hal/imu.h"
#include "ros/ros_module.h"

class ImuRos : public RosModule {
 public:
  ImuRos(Imu& imu, const char* topic, const char* frame, uint32_t periodMs);
  const char* name() const override { return "imu"; }
  size_t executorHandles() const override { return 0; }   // publisher only
  bool create(rcl_node_t* node, rclc_executor_t*) override {
    pub_ = rcl_get_zero_initialized_publisher();
    ok_ = rclc_publisher_init_default(&pub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), topic_) == RCL_RET_OK;
    return ok_;
  }
  void destroy(rcl_node_t* node) override { if (ok_) rcl_publisher_fini(&pub_, node); ok_ = false; }
  void update(uint32_t nowMs) override {
    if (nowMs - last_ < periodMs_) return;
    last_ = nowMs;
    // copy imu_.latest() into msg_, stamp it with rmw_uros_epoch_nanos(), then:
    rcl_publish(&pub_, &msg_, nullptr);
  }
 private:
  Imu& imu_; const char* topic_; uint32_t periodMs_, last_ = 0;
  rcl_publisher_t pub_{}; bool ok_ = false;
  sensor_msgs__msg__Imu msg_{};   // set frame_id once in the constructor (see drivetrain_ros.cpp)
};
```

**`main.cpp`**

```cpp
Imu imu;
ImuRos imuRos(imu, "imu/data", "imu_link", 20);   // 50 Hz
...
if (!imu.begin(10)) fail();
rosManager.addModule(&imuRos);
```

Then on the Pi, fuse `/odom` and `/imu/data` with `robot_localization`, and set `publishTf = false` (section 5).

The sensor's own library can be added to `lib_deps` in `platformio.ini` (e.g. `adafruit/Adafruit BNO055`).

### Example: more drive motors (4WD skid steer)

Add rows to `config::kWheels`:

```cpp
inline constexpr WheelConfig kWheels[] = {
    {"left_front",  Side::Left,  {25, 26, 27, 0, false}, {34, 35, false}},
    {"left_rear",   Side::Left,  {...},                 {...}},
    {"right_front", Side::Right, {33, 32,  4, 1, true},  {36, 39, true}},
    {"right_rear",  Side::Right, {...},                 {...}},
};
```

Each needs unique pins, a unique LEDC channel and its own encoder. Limits: 6 wheels (`Drivetrain::kMaxWheels`), 8 encoders in total across the whole firmware, 16 LEDC channels. Re-calibrate the track width afterwards (section 9, step 3). `wheel_status` grows automatically to 3 values per wheel.

If a wheel has **no encoder**, it can't run its own PID. In that case, wire it in parallel with the encoded wheel on the same side, sharing one driver channel.

### Example: a non-drive motor (the shaker)

The shaker's BLDC motor isn't part of the drivetrain. Give it its own hardware class (its ESC or driver, a speed loop if needed) and its own module, e.g. `ShakerRos`, subscribing to `shaker/cmd` (`std_msgs/Float32`, frequency in Hz) and publishing `shaker/state`. The pattern is the same as the IMU's, with one executor handle for the subscription. You can reuse `Pid` from `control/` and `QuadratureEncoder` or `HBridgeMotor` from `hal/` if they fit the hardware.

### Resource limits to keep in mind

| Resource                        | Limit                                            | Where it is set                                     |
|---------------------------------|--------------------------------------------------|-----------------------------------------------------|
| Publishers per node (micro-ROS) | 10                                               | micro-ROS build (`colcon.meta`); drivetrain uses 3  |
| Subscriptions per node          | 5                                                | same; drivetrain uses 1                             |
| Services / clients              | 1 / 1                                            | same                                                |
| RosManager modules              | 8                                                | `RosManager::kMaxModules`                           |
| PCNT encoders                   | 8                                                | ESP32 hardware                                      |
| LEDC PWM channels               | 16                                               | ESP32 hardware                                      |
| Message size over serial        | a few KB (fragmented, reliable)                  | micro-ROS transport MTU × history                   |

### Message types micro-ROS doesn't include

micro_ros_platformio ships the standard `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `diagnostic_msgs` and so on, but not every package. Anything else (e.g. `tf2_msgs`, or your own custom messages for the shaker or for coordinating the two rovers) goes in `esp32/extra_packages/` as a normal ROS 2 interface package (`package.xml`, `CMakeLists.txt`, `msg/*.msg`). Every package there is built into the micro-ROS library. Use `extra_packages/tf2_msgs` as the template. After adding or changing one, run `pio run -t clean_microros` and build again.

The `tf2_msgs` copy contains only `TFMessage`. It is compatible with the real package on the Pi because ROS 2 Humble matches message types by name and field layout.

If a custom message is also used on the Pi, put the same package in the Pi's ROS workspace (`~/program/ros2_ws/src`), so both sides have the identical definition.

### More publishers or subscriptions

If you need more publishers or subscriptions, add a custom `colcon.meta` with higher `RMW_UXRCE_MAX_PUBLISHERS` / `RMW_UXRCE_MAX_SUBSCRIPTIONS` and point `board_microros_user_meta` in `platformio.ini` at it (see the micro_ros_platformio README), then `pio run -t clean_microros`.

When adding a task, give it a priority **below** the drivetrain (5) unless it is equally time-critical, and add a `vTaskDelay` / `vTaskDelayUntil` to every loop so lower-priority tasks can run.

---

## 11. Two rovers

Both rovers can run this firmware. To keep their topics apart, give each a namespace in `config::kRos`:

```cpp
"sensor_rover",   // namespace   → /sensor_rover/cmd_vel, /sensor_rover/odom, ...
```

Also prefix the TF frames in `config::kDrivetrainRos`, e.g. `"sensor_rover/odom"` and `"sensor_rover/base_link"`, so the two rovers' transforms don't overwrite each other on the shared `/tf` topic. Alternatively, put the rovers on different ROS domain IDs (`config::kRos` domain **and** `ROS_DOMAIN_ID` in the container). See also [docker.md §7](docker.md#7-two-rovers-on-the-same-network).

---

## 12. Troubleshooting

| Problem                                         | Check                                                                                            |
|-------------------------------------------------|--------------------------------------------------------------------------------------------------|
| LED keeps blinking (slowly)                     | Agent running? Right device and baud (`/dev/esp32`, 115200)? Press EN to reset the ESP32          |
| LED blinks fast                                 | Hardware init failed: wheel count 0 or > 6, invalid dimensions, or more than 8 encoders           |
| Agent shows connection, then nothing            | Domain ID mismatch between `config::kRos` and `ROS_DOMAIN_ID`                                    |
| `ros2 topic list` doesn't show topics           | Same domain ID and RMW on all machines? Run `ros2 daemon stop` and retry                         |
| Upload fails "port busy"                        | Stop the micro-ROS agent first                                                                   |
| Upload fails "wrong boot mode"                  | Hold BOOT while upload starts; check nothing pulls GPIO 0/2/12/15 at boot                         |
| Rover drives in circles / wrong way             | Section 9, step 1                                                                                |
| A wheel jumps to full speed                     | Encoder sign reversed (section 9, step 1)                                                        |
| Motors twitch or ESP32 resets when motors start | Power noise: shared ground, capacitors, separate supply for logic                                 |
| Odometry drifts when turning                    | Calibrate track width (section 9, step 3). Skid steer always slips; add an IMU for heading        |
| Rover stops while holding a teleop key          | Teleop publishing slower than 2 Hz. Raise its rate or increase `cmdTimeoutMs`                    |
| Build fails in micro-ROS step                   | `pio run -t clean_microros`, then build again; needs internet and `git` on the first build       |
