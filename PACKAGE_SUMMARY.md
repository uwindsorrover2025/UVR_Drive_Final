# diff_drive_can Package Summary

## What Was Created

### Package Structure
```
src/diff_drive_can/
├── diff_drive_can/
│   ├── __init__.py
│   ├── ps4_twist_node.py      # PS4 controller to Twist converter
│   └── twist_to_can_node.py   # Twist to CAN motor commands
├── launch/
│   └── diff_drive_can.launch.py
├── config/
│   └── diff_drive_params.yaml
├── scripts/
│   └── test_can_communication.py
├── resource/
│   └── diff_drive_can
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── DESIGN.md
├── TESTING_GUIDE.md
└── PACKAGE_SUMMARY.md
```

### Key Features Implemented

1. **PS4 Controller Support**
   - Left stick controls linear/angular velocity
   - L1 button for emergency stop
   - R1 button for turbo mode
   - Configurable deadzone

2. **Safety Features**
   - Emergency stop toggle
   - Command timeout (0.5s default)
   - Acceleration ramping
   - CAN error recovery

3. **Differential Drive Kinematics**
   - 4ft (1.22m) wheel base
   - Max speeds: 2.0 m/s linear, 1.64 rad/s angular
   - PWM range: 0-1999 (1000 = neutral)

4. **CAN Communication**
   - SocketCAN interface (can0)
   - 1000kbps bitrate
   - CAN ID: 0x123
   - 4-byte message: [left_pwm_high, left_pwm_low, right_pwm_high, right_pwm_low]

## Quick Test

```bash
# Terminal 1: Build and launch
cd ~/Desktop/mars_rover_ws
colcon build --packages-select diff_drive_can
source install/setup.bash
ros2 launch diff_drive_can diff_drive_can.launch.py

# Terminal 2: Monitor CAN
candump can0

# Terminal 3: Monitor topics
source ~/Desktop/mars_rover_ws/install/setup.bash
ros2 topic echo /cmd_vel
```

## Configuration

All parameters can be adjusted in `config/diff_drive_params.yaml`:
- Robot dimensions
- Speed limits
- Control mappings
- Safety parameters
- CAN settings

## Next Steps

1. **Hardware Setup**
   - Connect CAN transceiver to Jetson pins 29, 31, 30
   - Add 120Ω termination resistors
   - Connect to STM32 CAN bus

2. **Testing**
   - Test emergency stop functionality
   - Verify differential drive calculations
   - Tune acceleration ramping
   - Test with actual motors

3. **Future Enhancements**
   - Add encoder feedback from STM32
   - Implement odometry publishing
   - Add battery monitoring
   - Create web-based diagnostics

## Important Notes

- STM32 code is already implemented and expects the CAN message format we're sending
- Always test emergency stop before operating
- Monitor CAN bus for errors during initial testing
- Adjust speed limits based on your rover's capabilities

## Support

For issues or questions:
- Check TESTING_GUIDE.md for troubleshooting
- Review DESIGN.md for technical details
- Monitor ROS logs for error messages
