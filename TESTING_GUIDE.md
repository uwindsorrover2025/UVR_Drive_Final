# Testing Guide for diff_drive_can Package

## Quick Start Commands

### 1. Build and Source
```bash
cd ~/Desktop/mars_rover_ws
colcon build --packages-select diff_drive_can
source install/setup.bash
```

### 2. Setup CAN Interface (if not already done)
```bash
# Load CAN modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Configure CAN interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can0 down

# Verify
ip -details link show can0
```

### 3. Launch the System 
```bash
# Main launch command
ros2 launch diff_drive_can diff_drive_can.launch.py

# With custom joystick device
ros2 launch diff_drive_can diff_drive_can.launch.py joy_dev:=/dev/input/js1
```

### 4. Monitor Topics (in separate terminals)
```bash
# Source workspace first in each terminal
source ~/Desktop/mars_rover_ws/install/setup.bash

# Monitor joystick input
ros2 topic echo /joy

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor topic rates
ros2 topic hz /joy
ros2 topic hz /cmd_vel
```

### 5. Monitor CAN Bus (in separate terminal)
```bash
# Monitor all CAN traffic
candump can0

# Monitor with timestamps
candump -ta can0

# Monitor specific CAN ID
candump can0,123:7FF
```

### 6. Test CAN Communication (without ROS)
```bash
# Run test script
python3 src/diff_drive_can/scripts/test_can_communication.py

# Manual CAN test
# Stop motors
cansend can0 123#03E803E8

# Forward 50%
cansend can0 123#05DC05DC

# Turn left
cansend can0 123#03E805DC
```

## PS4 Controller Test

1. Connect PS4 controller via USB or Bluetooth
2. Verify controller is detected:
   ```bash
   ls -l /dev/input/js*
   jstest /dev/input/js0
   ```

3. Launch the system and test controls:
   - **Left Stick Y**: Forward/backward
   - **Left Stick X**: Turn left/right
   - **L1**: Emergency stop (toggle)
   - **R1**: Turbo mode (hold)

## Troubleshooting

### CAN Interface Not Found
```bash
# Check if interface exists
ifconfig -a | grep can

# Check kernel modules
lsmod | grep can

# Check dmesg for errors
dmesg | grep -i can
```

### No Motor Response
1. Check CAN communication: `candump can0`
2. Verify STM32 is powered and connected
3. Check emergency stop status in ROS logs
4. Verify CAN termination resistors (120Ω)

### PS4 Controller Not Working
```bash
# Test raw joystick
jstest /dev/input/js0

# Check joy node
ros2 node list | grep joy
ros2 topic echo /joy
```

### Permission Issues
```bash
# Add user to dialout group for serial/CAN access
sudo usermod -a -G dialout $USER
# Logout and login again
```

## Expected CAN Output

When running correctly, `candump can0` should show:
```
can0  123   [4]  03 E8 03 E8    # Stop (1000, 1000)
can0  123   [4]  05 DC 05 DC    # Forward (1500, 1500)
can0  123   [4]  03 E8 05 DC    # Turn left (1000, 1500)
can0  123   [4]  05 DC 03 E8    # Turn right (1500, 1000)
```

## Safety Checks

Before operating:
1. Ensure emergency stop (L1) works
2. Test timeout (stop sending commands, motors should stop)
3. Verify PWM limits are correct for your motors
4. Have physical access to power switch
