# STM32 and ROS CAN Communication Compatibility Report

## Executive Summary

After analyzing both the STM32 firmware and ROS2 diff_drive_can package, I've identified one critical compatibility issue and one minor code ordering issue. The main issue is a **CAN bitrate mismatch** that has now been corrected in the ROS configuration.

## Compatibility Analysis

### ✅ **Compatible Components**

1. **CAN ID**: Both systems use `0x123`
   - STM32: `canfilter.FilterIdHigh = 0x123 << 5` (main.c:113)
   - ROS: `can_id: 0x123` (diff_drive_params.yaml:38)

2. **Data Format**: Identical 4-byte message structure
   - Bytes 0-1: Left motor PWM (big-endian int16)
   - Bytes 2-3: Right motor PWM (big-endian int16)
   - STM32: `(int16_t)((can_rx_data[0] << 8) | can_rx_data[1])` (main.c:481)
   - ROS: `struct.pack('>hh', left_pwm, right_pwm)` (twist_to_can_node.py:188)

3. **PWM Range**: Both use 0-1999
   - STM32: Clamps values to 0-1999 (main.c:485-486)
   - ROS: `pwm_min: 0`, `pwm_max: 1999` (diff_drive_params.yaml:32-33)

4. **PWM Timer Configuration**: STM32 Timer1 correctly configured
   - Prescaler: 167, Period: 1999 (main.c:318-320)
   - Generates 500Hz PWM signal with 0-1999 duty cycle range

### ❌ **Fixed Issues**

1. **CAN Bitrate Mismatch** (NOW FIXED)
   - STM32: Configured for 1 Mbps
     - APB1 clock: 42MHz (168MHz/4)
     - CAN timing: Prescaler=3, BS1=12TQ, BS2=1TQ
     - Bitrate = 42MHz / (3 × (1+12+1)) = 1 Mbps
   - ROS: Was 500 kbps, now updated to 1 Mbps in diff_drive_params.yaml

### ⚠️ **Minor Issues (Non-Critical)**

1. **STM32 CAN Filter Initialization Order**
   - Current order: Start CAN → Enable notifications → Configure filter
   - Recommended order: Configure filter → Start CAN → Enable notifications
   - This doesn't affect functionality but is better practice

## STM32 Code Recommendations

For better code organization in the STM32 firmware (main.c), consider reordering the CAN initialization:

```c
// USER CODE BEGIN 2
CAN_FilterTypeDef canfilter;
canfilter.FilterActivation = ENABLE;
canfilter.FilterBank = 0;
canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
canfilter.FilterIdHigh = 0x123 << 5;
canfilter.FilterIdLow = 0x0000;
canfilter.FilterMaskIdHigh = 0x7FF << 5;
canfilter.FilterMaskIdLow = 0x0000;
canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
canfilter.SlaveStartFilterBank = 14;

// Configure filter first
if (HAL_CAN_ConfigFilter(&hcan1, &canfilter) != HAL_OK)
{
    Error_Handler();
}

// Then start CAN
HAL_CAN_Start(&hcan1);

// Finally enable notifications
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
// USER CODE END 2
```

## Testing Recommendations

1. **Verify CAN Bus Configuration**:
   ```bash
   # On the ROS system, verify CAN interface is at 1 Mbps
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ip -details link show can0
   ```

2. **Test Communication**:
   ```bash
   # Use the provided test script
   cd ~/Desktop/mars_rover_ws
   python3 src/diff_drive_can/scripts/test_can_communication.py
   ```

3. **Monitor CAN Traffic**:
   ```bash
   # In another terminal
   candump can0
   ```

## System Integration Notes

### Motor Control Flow
1. PS4 controller → ps4_twist_node → /cmd_vel (Twist message)
2. twist_to_can_node subscribes to /cmd_vel
3. Converts linear/angular velocity to differential drive (left/right wheel velocities)
4. Maps velocities to PWM values (0-1999 range, 1000 = neutral)
5. Sends CAN message with ID 0x123 containing PWM values
6. STM32 receives CAN message and applies PWM to Timer1 channels

### Safety Features
- **Command timeout**: 0.5 seconds (both motors stop if no cmd_vel received)
- **PWM ramping**: 5 PWM units/ms for smooth acceleration
- **Emergency stop**: L1 button on PS4 controller
- **Deadzone**: 0.1 on joystick axes to prevent drift

## Conclusion

With the CAN bitrate now updated to 1 Mbps in the ROS configuration, the systems are fully compatible. The communication protocol, data format, and control ranges all match correctly. The minor STM32 code ordering issue doesn't affect functionality but could be improved for code clarity.

The system is ready for testing with the corrected configuration.