on konsole:

docker run -it --rm --privileged --network=host -v /dev:/dev -v /run/udev:/run/udev:ro -v /home/admin/ros2_ws:/root/ros2_ws my_robot

Usage inside Docker:
ros2 launch robot_bringup robot.launch.py

on another terminal
ros2 run joy joy_node

link
http://10.42.0.106:8080/

# Robot Operator USB — Setup Guide

## Step 4 — tmux Mouse Config

```bash
echo "set -g mouse on" > ~/.tmux.conf
```

---

## Step 5 — Create the Launch Script

```bash
cat > ~/Desktop/robot_start.sh << 'EOF'
#!/bin/bash
# robot_start.sh

PI_IP=$(docker context inspect remote --format '{{.Endpoints.docker.Host}}' | grep -oP '\d+\.\d+\.\d+\.\d+')

# Cleanup on Pi
docker --context remote stop my_robot 2>/dev/null
docker --context remote rm my_robot 2>/dev/null
ssh admin@$PI_IP "sudo pkill -f led_node 2>/dev/null; sudo pkill -f oakd_node 2>/dev/null"
sleep 3

tmux kill-session -t robot 2>/dev/null
tmux new-session -d -s robot -x 220 -y 50

# Pane 0: docker + launch
tmux send-keys -t robot:0 \
  "docker --context remote run -it --rm --privileged --network=host \
   --name my_robot \
   -v /dev:/dev \
   -v /run/udev:/run/udev:ro \
   -v /home/admin/ros2_ws:/root/ros2_ws \
   my_robot \
   bash -c 'source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 launch robot_bringup robot.launch.py'" Enter

# Pane 1: joy_node
tmux split-window -t robot:0 -v
tmux send-keys -t robot:0.1 "sleep 5 && ros2 run joy joy_node" Enter

# Pane 2: open dashboard in browser after delay
tmux split-window -t robot:0.1 -v
tmux send-keys -t robot:0.2 "sleep 10 && xdg-open http://${PI_IP}:8080" Enter

tmux attach -t robot
EOF

chmod +x ~/Desktop/robot_start.sh
```

---

## Step 6 — Create the Desktop Launcher

```bash
cat > ~/Desktop/robot_start.desktop << 'EOF'
[Desktop Entry]
Type=Application
Name=Start Robot
Exec=konsole -e bash -c '/home/imirdo/Desktop/robot_start.sh; exec bash'
Icon=utilities-terminal
Terminal=false
EOF

chmod +x ~/Desktop/robot_start.desktop
```

Then right-click `robot_start.desktop` → **Allow Launching**.

---

## Usage

- **Start**: double-click `Start Robot` on the desktop
- **Stop**: `Ctrl+C` in the top pane, then close Konsole
- **Reattach** (if detached): `tmux attach -t robot`
- **Detach** (keep running): `Ctrl+B` then `D`

---

## Notes

- Pi IP is resolved automatically from the `remote` docker context — no hardcoded IP
- Each run kills any leftover containers and orphaned `led_node` / `oakd_node` processes on the Pi before starting fresh
- Top pane = robot container logs, bottom pane = joy_node