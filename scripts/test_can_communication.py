#!/usr/bin/env python3
"""
Test script for CAN communication

Sends test motor commands to verify CAN bus functionality
"""

import can
import struct
import time
import sys


def test_can_communication(interface='can0', can_id=0x123):
    """Test CAN communication with various motor commands"""
    
    print(f"Testing CAN communication on {interface} with ID 0x{can_id:03X}")
    
    try:
        # Initialize CAN bus
        bus = can.interface.Bus(channel=interface, bustype='socketcan')
        print(f"✓ CAN interface {interface} initialized successfully")
        
    except Exception as e:
        print(f"✗ Failed to initialize CAN interface: {e}")
        print("\nTroubleshooting:")
        print("1. Check if CAN interface is up: sudo ip link set can0 up")
        print("2. Verify CAN modules loaded: lsmod | grep can")
        print("3. Check permissions: may need sudo")
        return 1
    
    # Test sequences
    test_sequences = [
    ("Stop (neutral)", 1500, 1500, 2.0),
    ("Forward 25%", 1625, 1625, 2.0),
    ("Forward 50%", 1750, 1750, 2.0),
    ("Forward 75%", 1875, 1875, 2.0),
    ("Forward 100%", 2000, 2000, 2.0),
    ("Turn left", 1625, 1750, 2.0),
    ("Turn right", 1750, 1625, 2.0),
    ("Spin left", 1000, 2000, 2.0),   # Full reverse left, full forward right
    ("Spin right", 2000, 1000, 2.0),  # Full forward left, full reverse right
    ("Reverse 25%", 1375, 1375, 2.0),
    ("Reverse 50%", 1250, 1250, 2.0),
    ("Reverse 75%", 1125, 1125, 2.0),
    ("Reverse 100%", 1000, 1000, 2.0),
    ("Stop (neutral)", 1500, 1500, 1.0),
]
    
    print("\nStarting test sequence...")
    print("Monitor with: candump can0")
    print("-" * 50)
    
    try:
        for description, left_pwm, right_pwm, duration in test_sequences:
            # Pack data as big-endian int16
            data = struct.pack('>hh', left_pwm, right_pwm)
            
            # Create CAN message
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=False
            )
            
            # Send message
            bus.send(msg, timeout=0.1)
            
            print(f"{description:20} | L={left_pwm:4d} R={right_pwm:4d} | "
                  f"Data: {data.hex().upper()}")
            
            time.sleep(duration)
            
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        
    except Exception as e:
        print(f"\n✗ Error during test: {e}")
        return 1
        
    finally:
        # Always send stop command before exiting
        print("\nSending stop command...")
        try:
            stop_data = struct.pack('>hh', 1000, 1000)
            stop_msg = can.Message(
                arbitration_id=can_id,
                data=stop_data,
                is_extended_id=False
            )
            bus.send(stop_msg, timeout=0.1)
            print("✓ Stop command sent")
        except:
            pass
            
        bus.shutdown()
        print("✓ CAN interface closed")
    
    print("\nTest completed successfully!")
    return 0


def main():
    """Main entry point"""
    # Parse command line arguments
    interface = 'can0'
    can_id = 0x123
    
    if len(sys.argv) > 1:
        interface = sys.argv[1]
    if len(sys.argv) > 2:
        can_id = int(sys.argv[2], 16)
    
    print("CAN Motor Test Script")
    print("=" * 50)
    print(f"Usage: {sys.argv[0]} [interface] [can_id_hex]")
    print(f"Example: {sys.argv[0]} can0 123")
    print("=" * 50)
    
    return test_can_communication(interface, can_id)


if __name__ == '__main__':
    sys.exit(main())