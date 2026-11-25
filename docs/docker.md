# Rover Docker Image

This document explains the Docker image in [`../docker`](../docker): what is inside it, how to build and run it, and how the udev rules make the rover's USB devices (ESP32 motor controller and RPLidar) show up under stable names.

---

## 1. Overview

The image is a complete ROS 2 Humble development environment that runs on the rover's Raspberry Pi (arm64) or on a normal PC (amd64). Once the container is running you get:

| Service       | Port   | How to open it                          | Purpose                                   |
|---------------|--------|-----------------------------------------|-------------------------------------------|
| code-server   | `8080` | `http://<pi-ip>:8080` in a browser       | VS Code in the browser, with terminals    |
| KasmVNC       | `8444` | `http://<pi-ip>:8444` in a browser       | XFCE desktop for RViz2, Gazebo, rqt       |
| SSH           | `2222` | `ssh -p 2222 robot@<pi-ip>`             | Terminal access into the container        |

The container uses the **host network**, so these ports are opened directly on the Pi's IP address. ROS 2 nodes in the container can talk to nodes on the Pi, on other containers, and on other machines on the same network (e.g. the second rover or a laptop).

### Files in `docker/`

| File                  | Where it is used        | What it does                                                        |
|-----------------------|-------------------------|---------------------------------------------------------------------|
| `Dockerfile`          | build                   | Defines the image                                                    |
| `start_services.sh`   | container start         | Starts SSH, KasmVNC and code-server                                  |
| `bashrc`              | copied to `~/.bashrc`   | Sources ROS 2, the micro-ROS agent and your workspace; adds aliases  |
| `kasmvnc.yaml`        | copied to `~/.vnc/`     | KasmVNC settings (port, resolution, HTTP instead of HTTPS)           |
| `99-rover.rules`      | **the Pi host**, not the image | udev rules giving USB devices fixed names (see section 6)      |
| `docker-compose.yml`  | optional                | Compose version of the `docker run` command in section 4             |

---

## 2. What is inside the image

Base: **Ubuntu 22.04 (Jammy)**, the Ubuntu release ROS 2 Humble is built for.

**Development tools**
- `build-essential`, `cmake`, `clang`, `git`, `git-lfs`, `python3`, `pip`, `pyserial`
- Editors: `nano`, `vim-tiny`, code-server
- Serial/USB debugging: `minicom`, `screen`, `usbutils` (`lsusb`), `v4l-utils` (cameras), `udev` (`udevadm`)
- Networking: `iproute2`, `net-tools`, `ping`, SSH client and server

**Desktop**
- XFCE4 desktop served through KasmVNC, so GUI tools work from any browser

**ROS 2 Humble**
- `ros-humble-desktop` (includes RViz2, rqt, demos) and `ros-dev-tools` (colcon, rosdep, vcstool)
- Robot description: `xacro`, `joint-state-publisher(-gui)`, `tf2-tools`
- Navigation and mapping: `navigation2`, `nav2-bringup`, `slam-toolbox`, `robot-localization`, `twist-mux`
- Control: `ros2-control`, `ros2-controllers`
- Simulation: `ros-gz` and `gz-ros2-control` (Gazebo Fortress, the version paired with Humble)
- Teleop: `teleop-twist-keyboard`, `teleop-twist-joy`, `joy`
- Sensors: `rplidar-ros`, `v4l2-camera`, `image-transport-plugins`
- Optional DDS: `rmw-cyclonedds-cpp`

**micro-ROS agent**
- Built from source into `/opt/microros_ws`. It is the bridge between ROS 2 and the micro-ROS firmware on the ESP32.

**User**
- User `robot`, password `robot`, uid `1000` (the same uid as the default user on Raspberry Pi OS, so files in the mounted folder have matching ownership).
- Groups: `sudo` (passwordless), `dialout` (serial ports), `video` (cameras), `plugdev` (raw USB).

---

## 3. Building the image

From the repository root:

```bash
docker build -t rover-dev:latest docker/
```

Build it **on the machine that will run it**. Docker builds for the CPU it runs on, so building on the Pi gives an arm64 image and building on a PC gives an amd64 image. Every downloaded binary (code-server, KasmVNC) is chosen with `dpkg --print-architecture`, so the same Dockerfile works for both.

The build takes a long time on a Pi, mostly because of the micro-ROS agent compile and the size of the ROS desktop and Gazebo packages. Expect an image of several GB.

### Pinning versions (optional)

By default code-server and KasmVNC are installed at the newest GitHub release. For reproducible builds, pin them:

```bash
docker build -t rover-dev:latest \
  --build-arg CODE_SERVER_VERSION=4.x.y \
  --build-arg KASMVNC_VERSION=1.x.y \
  docker/
```

The "latest" lookup uses the GitHub API, which allows 60 anonymous requests per hour. If a build fails at the code-server or KasmVNC step, pinning the version avoids the API call.

---

## 4. Running the container

Create the folder that holds your code first, and give it to uid 1000. If Docker creates it for you, it is owned by root and code-server cannot write to it.

```bash
mkdir -p ~/program
sudo chown 1000:1000 ~/program
```

Then start the container:

```bash
docker run -d \
  --name rover \
  --restart unless-stopped \
  --network host \
  --ipc host \
  --privileged \
  -e PASSWORD=change-me \
  -e ROS_DOMAIN_ID=0 \
  -v ~/program:/home/robot/program \
  -v /dev:/dev \
  -v /run/udev:/run/udev:ro \
  rover-dev:latest
```

### What each option does

| Option                         | Why it is needed                                                                                       |
|--------------------------------|--------------------------------------------------------------------------------------------------------|
| `--restart unless-stopped`     | Container starts again after the Pi reboots                                                             |
| `--network host`               | Container shares the Pi's IP; ROS 2 discovery works with other machines without port mapping           |
| `--ipc host`                   | Lets the DDS shared-memory transport work between the container and other ROS 2 processes on the Pi    |
| `--privileged`                 | Gives access to all devices, including USB devices plugged in after the container started               |
| `-e PASSWORD=...`              | Password for code-server and KasmVNC. **Change it.**                                                    |
| `-e VNC_PASSWORD=...`          | Optional separate VNC password (defaults to `PASSWORD`)                                                 |
| `-e ROS_DOMAIN_ID=...`         | ROS 2 domain (see section 7)                                                                            |
| `-v ~/program:/home/robot/program` | Your code and code-server settings live on the Pi and survive rebuilding the image               |
| `-v /dev:/dev`                 | The Pi's devices, including the udev symlinks `/dev/esp32` and `/dev/rplidar`, appear in the container  |
| `-v /run/udev:/run/udev:ro`    | Lets tools in the container read udev device info                                                       |

### Everyday commands

```bash
docker logs -f rover          # see startup output
docker exec -it rover bash    # open a shell in the container
docker stop rover             # stop
docker start rover            # start again
docker rm -f rover            # delete the container (your ~/program folder is kept)
```

To update after changing the Dockerfile: rebuild the image, `docker rm -f rover`, then run the `docker run` command again.

### What is kept and what is lost

- **Kept** (on the Pi in `~/program`): your ROS workspace, code, and code-server settings (`~/program/.settings`).
- **Lost when the container is deleted**: anything else written inside the container, such as `apt install`ed packages, code-server extensions installed outside `.settings`, and the VNC password file. Put permanent changes in the Dockerfile instead.

---

## 5. Using the container

### Startup sequence (`start_services.sh`)

1. Starts the SSH server on port 2222. It is 2222 rather than 22 so it does not clash with the Pi's own SSH server, because both share the same network.
2. On first start, creates the KasmVNC login (user `robot`, password `VNC_PASSWORD`).
3. Removes stale X11 lock files left behind if the container was stopped abruptly, then starts the VNC desktop on display `:1`.
4. Starts code-server in the foreground. This keeps the container running; if code-server exits, the container stops.

### Shell environment (`bashrc`)

Every interactive bash shell (code-server terminals, SSH, `docker exec`) automatically:

- sources `/opt/ros/humble/setup.bash`
- sources the micro-ROS agent (`/opt/microros_ws/install/local_setup.bash`)
- sources your workspace `~/program/ros2_ws/install/local_setup.bash` if it has been built
- sets `DISPLAY=:1`, so `rviz2`, `rqt` or Gazebo launched from any terminal open in the VNC desktop

Put your ROS 2 workspace at `~/program/ros2_ws` so it is picked up and survives rebuilds:

```bash
mkdir -p ~/program/ros2_ws/src
cd ~/program/ros2_ws
colcon build
source install/local_setup.bash
```

### Aliases

| Alias          | Command                                                                     |
|----------------|-----------------------------------------------------------------------------|
| `agent_serial` | `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 -b 115200` |
| `agent_udp`    | `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`                  |
| `ll`           | `ls -alF`                                                                    |

Use `agent_serial` when the ESP32 is connected by USB, and `agent_udp` when the ESP32 uses micro-ROS over Wi-Fi. The baud rate must match the one in the ESP32 firmware.

### Switching DDS implementation

Fast DDS is the default. To use Cyclone DDS (often smoother with Nav2 on Humble), uncomment this line in `bashrc` (or export it in a terminal):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Every ROS 2 node that needs to talk to each other, including those on the other rover or a laptop, should use the same RMW.

---

## 6. udev rules (`99-rover.rules`)

### The problem these rules solve

When a USB-serial device is plugged into Linux, the kernel names it `/dev/ttyUSB0`, `/dev/ttyUSB1`, ... in the order the devices are detected. That order is **not guaranteed**: it can change on every reboot or replug. If the ESP32 is `/dev/ttyUSB0` today and `/dev/ttyUSB1` tomorrow, the micro-ROS agent will open the lidar instead of the motor controller.

udev is the Linux service that reacts when devices appear. A udev rule can recognise a specific device by its properties and create a **fixed symlink** for it. With these rules:

- the ESP32 motor controller is always `/dev/esp32`
- the RPLidar is always `/dev/rplidar`

no matter which `ttyUSB` number the kernel gives them. Launch files and aliases then use these names and never need editing.

### Why the rules go on the Pi, not in the container

udev runs on the **host** (the Pi). The container does not run its own udev service. The Pi's udev creates the symlinks in the Pi's `/dev`, and because the container mounts `-v /dev:/dev`, the symlinks appear inside the container too, and they keep working when a device is unplugged and plugged back in.

### Anatomy of a rule

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="XXXX", GROUP="dialout", MODE="0660", SYMLINK+="esp32"
```

A rule is one line of comma-separated keys. Keys with `==` are **match conditions**: all of them must be true for the rule to apply. Keys with `=` or `+=` are **actions** that run when the rule matches.

| Key                         | Type   | Meaning                                                                                               |
|-----------------------------|--------|-------------------------------------------------------------------------------------------------------|
| `SUBSYSTEM=="tty"`          | match  | Only applies to serial/terminal devices (so it matches `/dev/ttyUSB*`, not the raw USB device)        |
| `ATTRS{idVendor}=="10c4"`   | match  | USB vendor ID of the USB-serial chip. `10c4` = Silicon Labs                                            |
| `ATTRS{idProduct}=="ea60"`  | match  | USB product ID. `10c4:ea60` = CP2102/CP2104 USB-to-UART chip                                           |
| `ATTRS{serial}=="XXXX"`     | match  | The chip's serial number, unique per device (on most chips)                                           |
| `KERNELS=="1-1.2"`          | match  | The physical USB port path (used in the CH340 rule)                                                    |
| `GROUP="dialout"`           | action | Device belongs to the `dialout` group; the container user `robot` is in this group                     |
| `MODE="0660"`               | action | Read/write for owner and group, nothing for others                                                    |
| `SYMLINK+="esp32"`          | action | Adds the symlink `/dev/esp32`. `+=` adds to the list instead of replacing other symlinks              |

`ATTRS` (with an S) means "this attribute on the device **or any parent device**". The tty device itself does not have a vendor ID; its parent USB device does. `ATTRS` lets the rule look up the device tree to find it.

The file name starts with `99-` because udev processes rules files in alphabetical order. `99` runs after the system's default rules, so the system defaults are not overridden by accident.

### Why the serial number matters

Many ESP32 development boards use the **same** CP2102 chip (`10c4:ea60`) as the RPLidar's USB adapter. If the rules only matched vendor and product ID, both devices would match both rules, and `/dev/esp32` and `/dev/rplidar` could point to either one. The serial number is what tells them apart.

Some boards use a **CH340** chip (`1a86:7523`) instead. CH340 chips have **no serial number**, so the only reliable way to tell one apart is by which physical USB port it is plugged into (`KERNELS`). This is what the commented-out third rule is for. The trade-off: that device must always be plugged into the same USB port on the Pi.

### Step-by-step: filling in the rules

**1. Plug in one device at a time** and find its name:

```bash
ls /dev/ttyUSB*
# or watch it appear:
dmesg -w     # then plug in the device; look for "now attached to ttyUSB0"
```

**2. Read its attributes:**

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial|KERNELS'
```

This prints the attributes of the device and all its parents, from the device upward. Use the values from the **first** block that has `idVendor`, which is the USB device itself. Example output:

```
    KERNELS=="1-1.2"
    ATTRS{idProduct}=="ea60"
    ATTRS{idVendor}=="10c4"
    ATTRS{serial}=="0001"
```

**3. Repeat for the other device.**

**4. Choose the matching method:**

- **Different serial numbers** for the two devices → replace `XXXX` (ESP32) and `YYYY` (RPLidar) in the file with them.
- **Same serial numbers.** Cheap boards often all report `0001`, which makes the serial useless. Match by port instead: delete the `ATTRS{serial}==` part and add `KERNELS=="<value>"`, using the `1-1.x` style value from step 2.
- **CH340 board** (`1a86:7523`) → comment out the CP2102 ESP32 rule and uncomment the CH340 rule, filling in `KERNELS`.
- If the vendor/product IDs from step 2 are something else, put those values in the rule instead.

Example, matching by port because both serials are `0001`:

```
SUBSYSTEM=="tty", KERNELS=="1-1.2", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0660", SYMLINK+="esp32"
SUBSYSTEM=="tty", KERNELS=="1-1.3", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0660", SYMLINK+="rplidar"
```

Port paths change if you move a device to a different port **or plug it in through a USB hub**. Label the ports on the rover.

**5. Install the rules on the Pi:**

```bash
sudo cp docker/99-rover.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**6. Check the symlinks:**

```bash
ls -l /dev/esp32 /dev/rplidar
# lrwxrwxrwx 1 root root 7 ... /dev/esp32 -> ttyUSB0
# lrwxrwxrwx 1 root root 7 ... /dev/rplidar -> ttyUSB1
```

Check inside the container too: `docker exec rover ls -l /dev/esp32 /dev/rplidar`.

### Troubleshooting the rules

| Symptom                                   | Likely cause / fix                                                                                   |
|-------------------------------------------|------------------------------------------------------------------------------------------------------|
| Symlink does not appear                   | A value does not match. Re-run `udevadm info -a` and compare every value character by character      |
| Symlink points to the wrong device        | Both devices match the same rule (usually identical serials). Switch to `KERNELS` matching           |
| Worked, stopped after moving the cable    | Rule uses `KERNELS` and the device is in a different port or hub                                     |
| `Permission denied` opening the port      | Check `GROUP="dialout"` is in the rule and `ls -l /dev/ttyUSB0` shows group `dialout`                |
| Need to see what udev is doing            | `udevadm test $(udevadm info -q path -n /dev/ttyUSB0) 2>&1 \| grep -i symlink`                       |
| Live view while plugging in               | `udevadm monitor --udev --property`                                                                  |

### Using the names

```bash
# micro-ROS agent for the ESP32
agent_serial

# RPLidar driver
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/rplidar
```

(Pick the launch file that matches your lidar model, e.g. `rplidar_a2m8_launch.py`, `rplidar_c1_launch.py`.)

---

## 7. Two rovers on the same network

Both rovers run this image. With the host network and the same `ROS_DOMAIN_ID`, every node on both rovers sees every other node. That is what lets the rovers cooperate, but standard topic names like `/cmd_vel`, `/scan`, `/odom` and `/tf` will **collide**: a velocity command meant for one rover will drive both.

Choose one approach:

- **Same domain, namespaced** (recommended when the rovers must exchange data): launch each rover's nodes in a namespace, e.g. `/sensor_rover/...` and `/shaker_rover/...`, and use frame prefixes for TF.
- **Separate domains** (fully isolated): run each rover with a different `-e ROS_DOMAIN_ID=` (e.g. `1` and `2`). Nodes in different domains cannot see each other at all, so data has to be bridged explicitly.

A laptop running RViz must use the same domain ID (and the same RMW) as the rover it wants to see.

---

## 8. Security notes

- Change `PASSWORD` from the default before running on a shared network.
- The user `robot` has the fixed password `robot` and passwordless `sudo`. Anyone who can SSH to port 2222 can do anything in the container. And because the container is `--privileged` with `/dev` mounted, they effectively have root on the Pi.
- code-server and KasmVNC are served over plain HTTP. That is fine on a private robot network, but do not expose these ports to the internet.

---

## 9. Troubleshooting

| Problem                                         | Fix                                                                                          |
|-------------------------------------------------|----------------------------------------------------------------------------------------------|
| Container exits immediately                     | `docker logs rover`. Usually code-server cannot write `~/program/.settings` → `sudo chown -R 1000:1000 ~/program` on the Pi |
| Port already in use                             | Something on the Pi already uses 8080, 8444 or 2222. Stop it or change the port in `start_services.sh` / `kasmvnc.yaml` |
| VNC page loads but desktop is black             | `docker exec rover vncserver -kill :1`, then `docker restart rover`                           |
| Changed `PASSWORD` but VNC still uses the old one | The VNC password is only set on first start. Recreate the container (`docker rm -f rover` and run again) |
| RViz/Gazebo window does not appear              | Run it from a container terminal (`DISPLAY=:1` is set there) and look in the VNC desktop      |
| Nodes on the laptop and rover cannot see each other | Same `ROS_DOMAIN_ID`? Same RMW? Same network/subnet? Firewall blocking multicast?          |
| Build fails downloading code-server / KasmVNC   | GitHub API rate limit, so pin the version with `--build-arg` (section 3)                      |
