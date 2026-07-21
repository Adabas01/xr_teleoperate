# G1-D Pico Teleop Quickstart

Guide for running `xr_teleoperate` on a real Unitree G1-D with Pico controllers
and Dex1-1 grippers.

Start with `pass-through` mode first. Use `immersive` or `ego` mode only after
the robot camera stream works in the Pico browser.

## Contents

- [What This Guide Covers](#what-this-guide-covers)
- [Hardware and Network](#hardware-and-network)
- [Install on a New Laptop](#install-on-a-new-laptop)
- [Start the Dex1-1 Gripper Server](#start-the-dex1-1-gripper-server)
- [Run Pass-Through Teleop](#run-pass-through-teleop)
- [Run Immersive or Ego Teleop](#run-immersive-or-ego-teleop)
- [Controls](#controls)
- [Troubleshooting](#troubleshooting)

## What This Guide Covers

This is the real-robot setup. You can skip Isaac Sim on the new laptop.

| Component | Used For |
| --- | --- |
| `xr_teleoperate` | Pico controller teleoperation |
| `unitree_sdk2_python` | Unitree DDS/RPC communication |
| Dex1-1 gripper server | Left and right gripper commands |
| Tele Imager | Robot camera stream for `immersive` and `ego` modes |
| G1-D AGV backend | Wheel base and column joystick control |

The G1-D wheel base and column control require the branch/fork that contains
`--motion-base=g1d_agv`.

## Hardware and Network

### Required Hardware

| Item | Notes |
| --- | --- |
| Ubuntu/Linux laptop | With Miniconda or Mambaforge |
| Ethernet or USB-Ethernet | Direct robot control link |
| Pico headset/controllers | Same Wi-Fi as the laptop |
| Unitree G1-D | Powered safely |
| Dex1-1 grippers | Server must run on the robot |

### Network Shape

Robot control uses Ethernet:

| Device | Interface | Address |
| --- | --- | --- |
| Laptop | `<ROBOT_ETH_IFACE>` | `192.168.123.x/24` |
| Robot/PC2 | Ethernet | `192.168.123.164` |

Pico/Vuer uses Wi-Fi:

| Device | Requirement |
| --- | --- |
| Laptop Wi-Fi | Same Wi-Fi as Pico |
| Pico | Same Wi-Fi as laptop |
| Robot Wi-Fi | Required only for `immersive` and `ego` modes |

Working examples from one laptop:

| Item | Value |
| --- | --- |
| Laptop Ethernet | `enp3s0` |
| Laptop Wi-Fi | `wlo1` |
| Robot Ethernet IP | `192.168.123.164` |
| Robot Wi-Fi IP | `192.168.212.34` |
| Robot/Pico camera Wi-Fi | `Zyxelace378.speed` |

Your new laptop may use different interface names, for example `enx...` for a
USB-Ethernet adapter or `wlp...` for Wi-Fi. Find them first:

```bash
ip -br addr
```

Look for:

| What to Find | Typical Clue |
| --- | --- |
| `<ROBOT_ETH_IFACE>` | Wired/USB-Ethernet interface, often no IP until configured |
| `<LAPTOP_WIFI_IFACE>` | Interface with your Wi-Fi address, often `wlo1` or `wlp...` |

Example output:

```text
lo      UNKNOWN 127.0.0.1/8
enp3s0  UP      192.168.123.2/24
wlo1    UP      192.168.212.50/24
```

From this example:

```text
export ROBOT_ETH_IFACE=enp3s0
export LAPTOP_WIFI_IFACE=wlo1
export LAPTOP_WIFI_IP=192.168.212.50
```

If the Ethernet interface has no `192.168.123.x` address, add one. Replace
`enp3s0` above with your actual `<ROBOT_ETH_IFACE>` first:

```bash
sudo ip addr add 192.168.123.2/24 dev "$ROBOT_ETH_IFACE"
sudo ip link set "$ROBOT_ETH_IFACE" up
```

Test the robot Ethernet connection:

```bash
ping -c 3 192.168.123.164
ssh unitree@192.168.123.164
```

Find the laptop Wi-Fi IP for the Pico Vuer URL:

```bash
ip -4 -o addr show "$LAPTOP_WIFI_IFACE"
```

Save the address as `LAPTOP_WIFI_IP`.

For immersive/ego camera mode, also confirm that the laptop can reach the robot
over Wi-Fi:

```bash
ping -c 3 192.168.212.34
```

### If the Default Interface Names Do Not Work

Use these checks to find the correct names.

| Question | Command | What You Want |
| --- | --- | --- |
| What interfaces exist? | `ip -br addr` | List of Ethernet and Wi-Fi interfaces |
| Which one is Wi-Fi? | `nmcli device status` | Device type `wifi` |
| Which Wi-Fi network am I on? | `nmcli -t -f ACTIVE,SSID dev wifi \| grep '^yes'` | Same SSID as the Pico |
| Which one is USB-Ethernet? | unplug/replug adapter, then `ip -br link` | The interface that appears/disappears |

Rules of thumb:

- `ROBOT_ETH_IFACE` is the wired/USB-Ethernet interface connected to the robot.
- `ROBOT_ETH_IFACE` must have an address like `192.168.123.x/24`.
- `LAPTOP_WIFI_IFACE` is the interface connected to the same Wi-Fi as the Pico.
- `LAPTOP_WIFI_IP` is the Wi-Fi IP used in the Pico Vuer URL.
- `--network-interface` should use `ROBOT_ETH_IFACE`, not the Wi-Fi interface.
- `--img-server-ip` is `192.168.123.164` for pass-through and the robot Wi-Fi IP for immersive/ego camera mode.

## Install on a New Laptop

### 1. Create a Workspace

```bash
mkdir -p ~/g1d_teleop
cd ~/g1d_teleop
```

### 2. Clone XR Teleop

Use your fork or branch if you want G1-D wheel base and column joystick control:

```bash
git clone https://github.com/YOUR_USERNAME/xr_teleoperate.git
cd xr_teleoperate
git checkout g1d-agv-motion
git submodule update --init --depth 1
```

Use the official upstream only if you want arm and gripper teleop without
`--motion-base=g1d_agv`:

```bash
git clone https://github.com/unitreerobotics/xr_teleoperate.git
```

### 3. Create the Conda Environment

```bash
conda create -n g1d_teleop python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
conda activate g1d_teleop
```

### 4. Install XR Teleop Packages

```bash
cd ~/g1d_teleop/xr_teleoperate
pip install -r requirements.txt

cd ~/g1d_teleop/xr_teleoperate/teleop/televuer
pip install -e .

cd ~/g1d_teleop/xr_teleoperate/teleop/teleimager
pip install -e . --no-deps
```

### 5. Install Unitree SDK Python

```bash
cd ~/g1d_teleop
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

### 6. Disable User-Site Python Packages

This avoids accidental imports from `~/.local`.

```bash
conda activate g1d_teleop
conda env config vars set PYTHONNOUSERSITE=1
conda deactivate
conda activate g1d_teleop
```

### 7. Generate the Pico/Vuer HTTPS Certificate

```bash
cd ~/g1d_teleop/xr_teleoperate/teleop/televuer

openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
  -keyout key.pem \
  -out cert.pem

mkdir -p ~/.config/xr_teleoperate
cp cert.pem key.pem ~/.config/xr_teleoperate/
```

## Start the Dex1-1 Gripper Server

Run this on the robot:

```bash
ssh unitree@192.168.123.164
cd ~/dex1_1_service/bin
sudo ./dex1_1_gripper_server
```

Leave this terminal running.

Expected DDS topics:

| Side | Command Topic | State Topic |
| --- | --- | --- |
| Left | `rt/dex1/left/cmd` | `rt/dex1/left/state` |
| Right | `rt/dex1/right/cmd` | `rt/dex1/right/state` |

## Run Pass-Through Teleop

Use this as the first real-robot test. It does not need Tele Imager.

Run on the laptop:

```bash
conda activate g1d_teleop
cd ~/g1d_teleop/xr_teleoperate/teleop

export ROBOT_ETH_IFACE=enp3s0  # change if your laptop uses another name

PYTHONNOUSERSITE=1 python teleop_hand_and_arm.py \
  --input-mode=controller \
  --display-mode=pass-through \
  --arm=G1_29 \
  --ee=dex1 \
  --img-server-ip=192.168.123.164 \
  --network-interface="$ROBOT_ETH_IFACE" \
  --motion \
  --motion-base=g1d_agv \
  --motion-max-vx=0.10 \
  --motion-max-vyaw=0.10 \
  --g1d-column-scale=0.20
```

Open Vuer in the Pico browser:

```text
https://<LAPTOP_WIFI_IP>:8012/?ws=wss://<LAPTOP_WIFI_IP>:8012
```

Example:

```text
https://192.168.212.50:8012/?ws=wss://192.168.212.50:8012
```

Then:

1. Accept the certificate warning.
2. Click `Virtual Reality`.
3. Hold the Pico controllers near the robot initial arm pose.
4. Press `r` in the laptop terminal.
5. Press `q` in the laptop terminal for a clean exit.

## Run Immersive or Ego Teleop

Use this only after pass-through works.

`immersive` and `ego` modes need Tele Imager because the Pico receives the robot
camera stream.

### 1. Check Robot Wi-Fi

Run on the robot:

```bash
ssh unitree@192.168.123.164
ip -br address
nmcli -t -f ACTIVE,SSID dev wifi | grep '^yes'
```

Expected working setup:

```text
eth0   UP  192.168.123.164/24
wlan0  UP  192.168.212.34/24
yes:Zyxelace378.speed
```

The laptop, Pico, and robot Wi-Fi must all be on the same Wi-Fi network.

### 2. Start Tele Imager

Run on the robot:

```bash
ssh unitree@192.168.123.164
conda activate tv

sudo systemctl stop teleimager.service 2>/dev/null || true
pkill -TERM -f teleimager-server 2>/dev/null || true

cd ~/unitree_eai_environment/service/teleimager
teleimager-server
```

Leave this terminal running.

### 3. Test the Robot Camera in Pico

Open this directly in the Pico browser:

```text
https://192.168.212.34:60001
```

Accept the certificate warning and press `Start`. The head-camera image should
appear before you launch immersive or ego teleop.

### 4. Run Immersive Mode

Run on the laptop:

```bash
conda activate g1d_teleop
cd ~/g1d_teleop/xr_teleoperate/teleop

export ROBOT_ETH_IFACE=enp3s0  # change if your laptop uses another name

PYTHONNOUSERSITE=1 python teleop_hand_and_arm.py \
  --input-mode=controller \
  --display-mode=immersive \
  --arm=G1_29 \
  --ee=dex1 \
  --img-server-ip=192.168.212.34 \
  --network-interface="$ROBOT_ETH_IFACE" \
  --motion \
  --motion-base=g1d_agv \
  --motion-max-vx=0.10 \
  --motion-max-vyaw=0.10 \
  --g1d-column-scale=0.20
```

### 5. Run Ego Mode

Use the same command, changing only the display mode:

```bash
conda activate g1d_teleop
cd ~/g1d_teleop/xr_teleoperate/teleop

export ROBOT_ETH_IFACE=enp3s0  # change if your laptop uses another name

PYTHONNOUSERSITE=1 python teleop_hand_and_arm.py \
  --input-mode=controller \
  --display-mode=ego \
  --arm=G1_29 \
  --ee=dex1 \
  --img-server-ip=192.168.212.34 \
  --network-interface="$ROBOT_ETH_IFACE" \
  --motion \
  --motion-base=g1d_agv \
  --motion-max-vx=0.10 \
  --motion-max-vyaw=0.10 \
  --g1d-column-scale=0.20
```

The Pico Vuer URL still uses the laptop Wi-Fi IP:

```text
https://<LAPTOP_WIFI_IP>:8012/?ws=wss://<LAPTOP_WIFI_IP>:8012
```

## Controls

Hold the Pico controllers near the robot initial arm pose before pressing `r`.

| Input | Action |
| --- | --- |
| `r` in laptop terminal | Start teleop |
| `q` in laptop terminal | Clean exit |
| Left controller pose | Left arm |
| Right controller pose | Right arm |
| Left trigger | Left Dex1 gripper |
| Right trigger | Right Dex1 gripper |
| Left stick up/down | Wheel base forward/back |
| Left stick left/right | Wheel base yaw |
| Right stick up/down | Column up/down |
| Right stick left/right | Ignored |
| Both thumbsticks pressed | Stop base and column |
| Right controller A | Exit teleop |

## Troubleshooting

### Pico Opens Vuer but Camera Is Black

Check:

- Pico, laptop, and robot are on the same Wi-Fi.
- Tele Imager is running on the robot.
- Pico can open `https://192.168.212.34:60001`.
- The camera certificate warning was accepted in the Pico browser.

### Head Camera Page Does Not Open

On the robot:

```bash
ss -lntp | grep 60001
ip -4 -br address show wlan0
nmcli -t -f ACTIVE,SSID dev wifi | grep '^yes'
```

On the laptop:

```bash
ping -c 3 192.168.212.34
```

### Teleop Waits for Arm DDS

Check:

- `--network-interface=<ROBOT_ETH_IFACE>`
- laptop Ethernet has `192.168.123.x/24`
- robot Ethernet responds to ping
- robot is powered and in the expected control mode

Commands:

```bash
ip -br addr
ip -4 -o addr show "$ROBOT_ETH_IFACE"
ping -c 3 192.168.123.164
```

### Teleop Waits for Dex1 DDS

Restart the gripper server:

```bash
ssh unitree@192.168.123.164
cd ~/dex1_1_service/bin
sudo ./dex1_1_gripper_server
```

### Tele Imager Finds No Cameras

Stop old Tele Imager processes:

```bash
sudo systemctl stop teleimager.service 2>/dev/null || true
pkill -TERM -f teleimager-server 2>/dev/null || true
```

Reload the UVC driver:

```bash
sudo modprobe -r uvcvideo
sleep 2
sudo modprobe uvcvideo
sleep 3
```

Check cameras:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

Expected camera pages:

| Camera | URL |
| --- | --- |
| Head | `https://192.168.212.34:60001` |
| Left wrist | `https://192.168.212.34:60002` |
| Right wrist | `https://192.168.212.34:60003` |
