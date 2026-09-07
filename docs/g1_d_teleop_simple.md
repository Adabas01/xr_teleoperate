# G1-D Teleop Simple Runbook

This is the short "run it today" checklist for G1-D teleop with Pico controllers, Dex1 grippers, Tele Imager, AGV motion, side-trigger torso yaw, and torso yaw recording.

## Quick Run

Use these example values unless your network changed:

```text
Robot SSH IP: 192.168.123.164
Robot image server IP: 192.168.50.103
Laptop Wi-Fi IP: 192.168.50.240
Laptop robot interface: enp3s0
Wi-Fi SSID: ASUS_ROG_X
```

The laptop, robot, and Pico should be on the same Wi-Fi. The laptop also needs the wired robot connection.

### 1. Robot Terminal: Dex1 Service

Open terminal 1:

```bash
ssh unitree@192.168.123.164
```

If asked:

```text
ros:foxy(1) noetic(2) ?
```

enter:

```text
1
```

Start Dex1:

```bash
cd ~/dex1_1_service/bin
sudo ./dex1_1_gripper_server
```

Leave this running. Good sign:

```text
Dex1-1 Gripper Server started.
```

### 2. Robot Terminal: Tele Imager

Open terminal 2:

```bash
ssh unitree@192.168.123.164
```

If asked for ROS, enter:

```text
1
```

Start the image server:

```bash
teleimager-server
```

Leave this running. Good sign:

```text
[Image Server] Running... Press Ctrl+C to exit.
```

### 3. Laptop Terminal: Teleop

Open a new laptop terminal:

```bash
cd ~/adam_unitree/xr_teleoperate/teleop
conda activate g1d_teleop
```

This command records into a dedicated conveyor-box dataset directory. The `--task-goal` text becomes the LeRobot task prompt after conversion.

Run teleop:

```bash
PYTHONNOUSERSITE=1 python teleop_hand_and_arm.py \
  --input-mode=controller \
  --display-mode=ego \
  --arm=G1_29 \
  --ee=dex1 \
  --img-server-ip=192.168.50.103 \
  --network-interface=enp3s0 \
  --motion \
  --motion-base=g1d_agv \
  --motion-max-vx=0.10 \
  --motion-max-vyaw=0.10 \
  --g1d-column-scale=0.20 \
  --g1d-torso-yaw-input=side_triggers \
  --record \
  --record-torso-yaw \
  --task-dir="$HOME/adam_unitree/datasets/g1d_conveyor_green_box_raw" \
  --task-name="green_box_empty_to_yellow_discard_to_red" \
  --task-goal="Pick the green box from the conveyor with the right hand, empty it into the yellow bin on the left, then place the empty box into the red container on the right." \
  --task-desc="The robot stands in front of a conveyor belt. A green box arrives. The yellow bin is on the left and the red container is on the right." \
  --task-steps="1. Pick the green box from the conveyor with the right hand; 2. Use the left hand to empty the box into the yellow bin on the left; 3. Turn toward the red container on the right; 4. Place the empty box into the red container."
```

### 4. Pico Browser

Open this in the Pico browser:

```text
https://192.168.50.240:8012/?ws=wss://192.168.50.240:8012
```

If the laptop Wi-Fi IP changed, replace both `192.168.50.240` values.

Accept the browser certificate warning if it appears.

## Controls

Start/stop:

```text
Pico left X / keyboard r: start teleop
Pico left Y / keyboard s: start or save recording
Pico right A / keyboard q: stop and exit
```

Motion:

```text
Left joystick Y: AGV forward/back
Left joystick X: AGV yaw
Right joystick Y: G1-D column up/down
Back/index triggers: Dex1 grippers
Side/grip triggers: torso yaw
Press both thumbsticks: damping mode
```

## Recordings

With the command above, recordings are saved under:

```text
~/adam_unitree/datasets/g1d_conveyor_green_box_raw/green_box_empty_to_yellow_discard_to_red/episode_XXXX/data.json
```

Because `--record-torso-yaw` is enabled, each frame includes torso yaw data for later conversion and replay.

The most important task field is:

```text
--task-goal
```

That is the text the LeRobot converter stores as the `task` prompt. Keep it the same for all demonstrations of this exact task.

## Stop Everything

1. Stop teleop with Pico right A or keyboard `q`.
2. Stop `teleimager-server` with `Ctrl-C`.
3. Stop `dex1_1_gripper_server` with `Ctrl-C`.

## Finding IPs And Interfaces

Use this section only when the quick-run values do not work.

### Laptop Wi-Fi IP

Run on the laptop:

```bash
ifconfig
```

Look for the Wi-Fi interface, usually `wlo1`. Example:

```text
wlo1:
  inet 192.168.50.240
```

Use that IP in the Pico browser URL:

```text
https://<LAPTOP_WIFI_IP>:8012/?ws=wss://<LAPTOP_WIFI_IP>:8012
```

### Laptop Robot Interface

Run on the laptop:

```bash
ifconfig
ip route
```

Look for the wired robot interface on the `192.168.123.x` network. Example:

```text
enp3s0:
  inet 192.168.123.42
```

Use that interface in the teleop command:

```text
--network-interface=enp3s0
```

### Robot Image Server IP

The `--img-server-ip` value is the robot Wi-Fi IP, not the laptop IP.

From an SSH terminal on the robot:

```bash
ip route
hostname -I
```

Use the robot IP on the same Wi-Fi as the Pico/laptop. Example:

```text
--img-server-ip=192.168.50.103
```

## Caveats

The laptop, robot, and Pico must be on the same Wi-Fi for Pico browser video/control. Ideally use `ASUS_ROG_X`.

The laptop must also keep the wired robot connection active for Unitree DDS control. The teleop command should use the wired interface in `--network-interface`.

Keep both robot terminals running while teleop is active. Dex1 controls will not work without `dex1_1_gripper_server`, and ego/immersive video will not work without `teleimager-server`.

If the Dex1 service prints temporary motor timeout warnings but then prints `Dex1-1 Gripper Server started.`, it is ready.

If the Pico page does not open, re-check the laptop Wi-Fi IP and use `https`, not `http`.

If the camera stream does not connect, re-check the robot image server Wi-Fi IP used in `--img-server-ip`.

Record conveyor belt task:

```bash
PYTHONNOUSERSITE=1 python teleop_hand_and_arm.py \
  --input-mode=controller \
  --display-mode=ego \
  --arm=G1_29 \
  --ee=dex1 \
  --img-server-ip=192.168.50.103 \
  --network-interface=enp3s0 \
  --motion \
  --motion-base=g1d_agv \
  --motion-max-vx=0.10 \
  --motion-max-vyaw=0.10 \
  --g1d-column-scale=0.20 \
  --g1d-torso-yaw-input=side_triggers \
  --record \
  --record-torso-yaw \
  --task-name=conveyor \
  --task-dir="$HOME/adam_unitree/datasets/g1d_conveyor_green_box_raw" \
  --task-name="green_box_empty_to_yellow_discard_to_red" \
  --task-goal="Pick the green box from the conveyor with the right hand, empty it into the yellow bin on the left, then place the empty box into the red container on the right." \
  --task-desc="The robot stands in front of a conveyor belt. A green box arrives. The yellow bin is on the left and the red container is on the right." \
  --task-steps="1. Pick the green box from the conveyor with the right hand; 2. Use the left hand to empty the box into the yellow bin on the left; 3. Turn toward the red container on the right; 4. Place the empty box into the red container."
```