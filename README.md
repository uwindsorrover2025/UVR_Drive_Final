# University-of-Windsor-Mars-Rover-Team-Drive Train System - 2025

## diff_drive_can

ROS 2 package for controlling a differential drive rover using a PS4 controller over CAN bus.

## Overview

This package provides a complete pipeline for controlling a differential drive robot:
1. Reads PS4 controller input via the `joy` package
2. Converts joystick input to `geometry_msgs/Twist` messages
3. Applies differential drive kinematics to calculate wheel speeds
4. Sends motor commands over CAN bus to an STM32 microcontroller

## Features

- **PS4 Controller Support**: Uses left stick for linear/angular velocity control
- **Safety Features**: 
  - Emergency stop (L1 button)
  - Command timeout (stops motors if no commands received)
  - Deadzone filtering
  - Acceleration ramping
- **Turbo Mode**: R1 button for increased speed
- **CAN Communication**: Sends motor commands via SocketCAN
- **Configurable Parameters**: All parameters can be adjusted via YAML config

## Installation

### Prerequisites

1. ROS 2 Humble installed
2. Python 3.8+
3. CAN interface hardware connected to Jetson

### Dependencies

Install required packages:
```bash
# ROS 2 dependencies
sudo apt update
sudo apt install ros-humble-joy

# Python CAN library
pip3 install python-can
```

### Building

```bash
cd ~/Desktop/mars_rover_ws
colcon build --packages-select diff_drive_can
source install/setup.bash
```

## Hardware Setup

### CAN Interface Configuration

1. Enable CAN kernel modules:
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
```

2. Configure CAN interface:
```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

3. Verify interface:
```bash
ip -details link show can0
```

### Jetson CAN Pins
- **CAN0_TX**: Pin 29 (GPIO01)
- **CAN0_RX**: Pin 31 (GPIO11)
- **GND**: Pin 30

Connect these to a CAN transceiver (e.g., MCP2551) with proper 120Ω termination.

## Usage

### Launch the System

```bash
ros2 launch diff_drive_can diff_drive_can.launch.py
```

With custom joystick device:
```bash
ros2 launch diff_drive_can diff_drive_can.launch.py joy_dev:=/dev/input/js1
```

### Monitor Topics

In separate terminals:
```bash
# Monitor joystick input
ros2 topic echo /joy

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor CAN bus
candump can0
```

## Configuration

Edit `config/diff_drive_params.yaml` to adjust parameters:

- **Robot dimensions**: `wheel_base` (default: 1.22m / 4ft)
- **Speed limits**: `max_linear_speed`, `max_angular_speed`
- **Control mapping**: Joystick axes and buttons
- **Safety parameters**: Timeout, ramping rates
- **CAN settings**: Interface name, message ID

## Controls

### PS4 Controller Mapping

- **Left Stick Y-axis**: Forward/backward movement
- **Left Stick X-axis**: Left/right turning
- **L1 Button**: Emergency stop (toggle)
- **R1 Button**: Turbo mode (hold)

## CAN Protocol

The package sends CAN messages with ID `0x123` containing 4 bytes:

```
Bytes 0-1: Left motor speed (int16_t, big-endian)
Bytes 2-3: Right motor speed (int16_t, big-endian)

Range: 0-1999 (0 = full reverse, 1000 = stop, 1999 = full forward)
```

## Testing

### Manual CAN Testing

Send test commands:
```bash
# Stop both motors
cansend can0 123#03E803E8

# Forward at 50% speed
cansend can0 123#05DC05DC

# Turn left
cansend can0 123#03E805DC
```

### Automated Testing

Run the test script:
```bash
python3 src/diff_drive_can/scripts/test_can_communication.py
```

## Troubleshooting

### CAN Interface Issues

If CAN interface fails to initialize:
```bash
# Check if interface exists
ifconfig -a | grep can

# Check kernel modules
lsmod | grep can

# Check for errors
dmesg | grep -i can
```

### PS4 Controller Not Detected

```bash
# List available joysticks
ls -l /dev/input/js*

# Test joystick
jstest /dev/input/js0
```

### No Motor Movement

1. Check CAN communication: `candump can0`
2. Verify STM32 is receiving messages
3. Check emergency stop status in logs
4. Ensure proper power to motor controllers

## Safety Notes

- Always have physical access to power switch
- Test emergency stop before operation
- Start with low speed limits during initial testing
- Monitor CAN bus for errors

## License

Apache-2.0

## Maintainer

Mars Rover Team - rover2025@mars.rover
