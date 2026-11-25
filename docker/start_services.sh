#!/bin/bash
# Starts SSH (port 2222), KasmVNC desktop (port 8444) and code-server (port 8080).
# Change the passwords with:  -e PASSWORD=...  (and optionally -e VNC_PASSWORD=...)
set -e

export PASSWORD="${PASSWORD:-robot}"
VNC_PASSWORD="${VNC_PASSWORD:-$PASSWORD}"

# --- SSH ---------------------------------------------------------------------
sudo service ssh start

# --- KasmVNC -----------------------------------------------------------------
# Create the VNC login on first start (user "robot", write access)
if [ ! -f "$HOME/.kasmpasswd" ]; then
  printf '%s\n%s\n' "$VNC_PASSWORD" "$VNC_PASSWORD" | kasmvncpasswd -u robot -w
fi

# Clean up stale locks left behind if the container was stopped hard
vncserver -kill :1 >/dev/null 2>&1 || true
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

vncserver :1
export DISPLAY=:1   # GUI apps (RViz, Gazebo) started from code-server terminals show up in the VNC desktop

# --- code-server (foreground; keeps the container running) -------------------
exec code-server \
  --bind-addr 0.0.0.0:8080 \
  --auth password \
  --user-data-dir /home/robot/program/.settings
