# PS4 to CAN Communication Troubleshooting Guide

## Problem Description
PS4 controller is recognized but commands are not reaching the CAN bus. Manual CAN commands work fine.

## Quick Diagnostic Steps

### 1. Build and Source the Package
```bash
cd ~/Desktop/mars_rover_ws
colcon build --packages-select diff_drive_can
source install/setup.bash
```

### 2. Run the Debug Launch File
```bash
ros2 launch diff_drive_can debug_ps4_can.launch.py
```

This will start all nodes with proper timing and show debug output.

### 3. In Another Terminal, Monitor CAN Traffic
```bash
candump can0
```

### 4. Check Individual Components

#### A. Verify PS4 Controller is Connected
```bash
# Check if controller is detected
ls -la /dev/input/js*

# Test raw joystick input
jstest /dev/input/js0
```

#### B. Check ROS Topics
```bash
# List all active topics
ros2 topic list

# Monitor joy messages
ros2 topic echo /joy

# Monitor cmd_vel messages
ros2 topic echo /cmd_vel

# Check topic info
ros2 topic info /joy
ros2 topic info /cmd_vel
```

#### C. Check Running Nodes
```bash
ros2 node list
```

You should see:
- `/joy_node`
- `/ps4_twist_node`
- `/twist_to_can_node`

## Common Issues and Solutions

### Issue 1: No /joy Messages
**Symptoms**: `ros2 topic echo /joy` shows nothing

**Solutions**:
1. Check controller connection:
   ```bash
   ls -la /dev/input/js*
   ```
2. Verify permissions:
   ```bash
   sudo chmod 666 /dev/input/js0
   ```
3. Test with jstest:
   ```bash
   jstest /dev/input/js0
   ```

### Issue 2: /joy Messages but No /cmd_vel
**Symptoms**: Joy messages appear but no cmd_vel messages

**Solutions**:
1. Check ps4_twist_node is running:
   ```bash
   ros2 node info /ps4_twist_node
   ```
2. Check parameter loading:
   ```bash
   ros2 param list /ps4_twist_node
   ```
3. Verify axis mapping (default is axes 0 and 1):
   ```bash
   ros2 param get /ps4_twist_node linear_axis
   ros2 param get /ps4_twist_node angular_axis
   ```

### Issue 3: /cmd_vel Messages but No CAN Traffic
**Symptoms**: cmd_vel messages appear but candump shows nothing

**Solutions**:
1. Check CAN interface status:
   ```bash
   ip link show can0
   ```
2. Ensure CAN is UP and configured for 1 Mbps:
   ```bash
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ```
3. Check twist_to_can_node logs for errors
4. Verify CAN parameters:
   ```bash
   ros2 param get /twist_to_can_node can_interface
   ros2 param get /twist_to_can_node can_bitrate
   ```

### Issue 4: Parameter Loading Issues
**Symptoms**: Nodes running but using default parameters

**Solution**: Launch with explicit config file:
```bash
ros2 launch diff_drive_can diff_drive_can.launch.py
```

## Manual Testing Steps

### 1. Test CAN Hardware
```bash
# Send test message
cansend can0 123#0003E70003E7

# Monitor in another terminal
candump can0
```

### 2. Test Each Node Individually

#### Test Joy Node Only
```bash
ros2 run joy joy_node
# In another terminal
ros2 topic echo /joy
```

#### Test PS4 Twist Conversion
```bash
# Start joy node
ros2 run joy joy_node

# In another terminal, start ps4_twist_node with params
ros2 run diff_drive_can ps4_twist_node --ros-args --params-file ~/Desktop/mars_rover_ws/install/diff_drive_can/share/diff_drive_can/config/diff_drive_params.yaml

# Monitor output
ros2 topic echo /cmd_vel
```

#### Test CAN Output
```bash
# Publish test cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"

# Start twist_to_can_node
ros2 run diff_drive_can twist_to_can_node --ros-args --params-file ~/Desktop/mars_rover_ws/install/diff_drive_can/share/diff_drive_can/config/diff_drive_params.yaml

# Monitor CAN
candump can0
```

## Debug Output Analysis

When running the debug script, look for:

1. **Node Status**: All three nodes should show "RUNNING"
2. **Message Counts**: Both /joy and /cmd_vel should increment
3. **CAN Interface**: Should show "UP"
4. **Warnings**: Pay attention to any timing or conversion warnings

## Advanced Debugging

### Enable Debug Mode
```bash
ros2 param set /ps4_twist_node debug_mode true
ros2 param set /twist_to_can_node debug_mode true
```

### Check System Logs
```bash
# Check kernel messages for CAN errors
dmesg | grep -i can

# Check system journal
journalctl -xe | grep -i can
```

### Monitor CAN Statistics
```bash
ip -s -d link show can0
```

## Expected Behavior

When everything works correctly:
1. Moving PS4 joysticks should generate /joy messages
2. /joy messages should be converted to /cmd_vel by ps4_twist_node
3. /cmd_vel messages should generate CAN messages with ID 0x123
4. candump should show messages like: `can0  123   [4]  03 E7 03 E7`

## Contact for Help

If issues persist after following this guide:
1. Save the output of the debug launch file
2. Save the output of `ros2 topic list` and `ros2 node list`
3. Save any error messages from the terminal
4. Check the STM32_ROS_COMPATIBILITY_REPORT.md for additional details