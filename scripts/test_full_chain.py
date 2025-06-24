#!/usr/bin/env python3
"""
Comprehensive test script for PS4 to CAN communication chain
"""

import subprocess
import time
import sys

def run_command(cmd, check=True):
    """Run a command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, check=check)
        return result.stdout, result.stderr, result.returncode
    except subprocess.CalledProcessError as e:
        return e.stdout, e.stderr, e.returncode

def check_ps4_controller():
    """Check if PS4 controller is connected"""
    print("\n=== Checking PS4 Controller ===")
    
    # Check for joystick device
    stdout, _, _ = run_command("ls -la /dev/input/js*", check=False)
    if "/dev/input/js0" in stdout:
        print("✓ PS4 controller found at /dev/input/js0")
        
        # Check permissions
        stdout, _, _ = run_command("ls -la /dev/input/js0")
        print(f"  Device info: {stdout.strip()}")
        
        # Test with jstest (brief)
        print("  Testing joystick input (move sticks)...")
        run_command("timeout 3 jstest /dev/input/js0", check=False)
        
        return True
    else:
        print("✗ No PS4 controller found!")
        print("  Make sure controller is connected via USB")
        return False

def check_can_interface():
    """Check CAN interface status"""
    print("\n=== Checking CAN Interface ===")
    
    stdout, stderr, code = run_command("ip link show can0", check=False)
    if code != 0:
        print("✗ CAN interface can0 not found!")
        print("  Setting up CAN interface...")
        
        # Try to set up CAN
        run_command("sudo ip link set can0 type can bitrate 1000000", check=False)
        run_command("sudo ip link set can0 up", check=False)
        
        # Check again
        stdout, stderr, code = run_command("ip link show can0", check=False)
        if code != 0:
            print("✗ Failed to set up CAN interface")
            return False
    
    if "UP" in stdout:
        print("✓ CAN interface is UP")
    else:
        print("⚠ CAN interface exists but is DOWN")
        print("  Bringing up CAN interface...")
        run_command("sudo ip link set can0 up", check=False)
    
    # Check bitrate
    stdout, _, _ = run_command("ip -details link show can0")
    if "bitrate 1000000" in stdout:
        print("✓ CAN bitrate is 1 Mbps")
    else:
        print("⚠ CAN bitrate may not be 1 Mbps")
        print("  Setting bitrate...")
        run_command("sudo ip link set can0 down", check=False)
        run_command("sudo ip link set can0 type can bitrate 1000000", check=False)
        run_command("sudo ip link set can0 up", check=False)
    
    return True

def test_manual_can():
    """Test manual CAN communication"""
    print("\n=== Testing Manual CAN Communication ===")
    
    print("Sending test CAN message...")
    stdout, stderr, code = run_command("cansend can0 123#0003E70003E7", check=False)
    
    if code == 0:
        print("✓ Manual CAN send successful")
        print("  If STM32 is connected, motors should move")
        return True
    else:
        print("✗ Manual CAN send failed")
        print(f"  Error: {stderr}")
        return False

def check_ros_nodes():
    """Check if ROS nodes are running"""
    print("\n=== Checking ROS Nodes ===")
    
    stdout, _, _ = run_command("ros2 node list")
    nodes = stdout.strip().split('\n')
    
    required_nodes = {
        '/joy_node': False,
        '/ps4_twist_node': False,
        '/twist_to_can_node': False
    }
    
    for node in nodes:
        if node in required_nodes:
            required_nodes[node] = True
    
    all_running = True
    for node, running in required_nodes.items():
        if running:
            print(f"✓ {node} is running")
        else:
            print(f"✗ {node} is NOT running")
            all_running = False
    
    return all_running

def check_ros_topics():
    """Check ROS topic connections"""
    print("\n=== Checking ROS Topics ===")
    
    # Check /joy topic
    stdout, _, _ = run_command("ros2 topic info /joy")
    if "Publisher count: 1" in stdout and "Subscription count:" in stdout:
        sub_count = int(stdout.split("Subscription count: ")[1].split()[0])
        if sub_count > 0:
            print("✓ /joy topic: 1 publisher, {} subscribers".format(sub_count))
        else:
            print("⚠ /joy topic: 1 publisher, but NO subscribers!")
    else:
        print("✗ /joy topic not properly connected")
    
    # Check /cmd_vel topic
    stdout, _, _ = run_command("ros2 topic info /cmd_vel")
    if "Publisher count:" in stdout and "Subscription count:" in stdout:
        pub_count = int(stdout.split("Publisher count: ")[1].split()[0])
        sub_count = int(stdout.split("Subscription count: ")[1].split()[0])
        if pub_count > 0 and sub_count > 0:
            print("✓ /cmd_vel topic: {} publishers, {} subscribers".format(pub_count, sub_count))
        else:
            print("⚠ /cmd_vel topic: {} publishers, {} subscribers".format(pub_count, sub_count))
    else:
        print("✗ /cmd_vel topic not properly connected")

def monitor_topics():
    """Monitor topic data flow"""
    print("\n=== Monitoring Topic Data (5 seconds) ===")
    print("Move the PS4 controller sticks...")
    
    # Start monitoring in background
    joy_proc = subprocess.Popen(
        "timeout 5 ros2 topic hz /joy",
        shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    
    cmd_vel_proc = subprocess.Popen(
        "timeout 5 ros2 topic hz /cmd_vel",
        shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    )
    
    # Wait for monitoring to complete
    joy_out, _ = joy_proc.communicate()
    cmd_vel_out, _ = cmd_vel_proc.communicate()
    
    # Parse results
    if "average rate:" in joy_out:
        rate = joy_out.split("average rate: ")[1].split()[0]
        print(f"✓ /joy messages received at {rate} Hz")
    else:
        print("✗ No /joy messages received!")
    
    if "average rate:" in cmd_vel_out:
        rate = cmd_vel_out.split("average rate: ")[1].split()[0]
        print(f"✓ /cmd_vel messages received at {rate} Hz")
    else:
        print("✗ No /cmd_vel messages received!")

def monitor_can():
    """Monitor CAN bus for messages"""
    print("\n=== Monitoring CAN Bus (5 seconds) ===")
    print("Move the PS4 controller sticks...")
    
    stdout, _, _ = run_command("timeout 5 candump -n 10 can0", check=False)
    
    if stdout.strip():
        lines = stdout.strip().split('\n')
        msg_count = len(lines)
        print(f"✓ Received {msg_count} CAN messages")
        print("  Sample messages:")
        for line in lines[:3]:
            print(f"    {line}")
    else:
        print("✗ No CAN messages received!")

def main():
    print("PS4 to CAN Communication Chain Test")
    print("=" * 50)
    
    # Check each component
    ps4_ok = check_ps4_controller()
    can_ok = check_can_interface()
    
    if not ps4_ok or not can_ok:
        print("\n⚠ Basic hardware checks failed. Fix these issues first.")
        return
    
    # Test manual CAN
    test_manual_can()
    
    # Check ROS components
    nodes_ok = check_ros_nodes()
    
    if not nodes_ok:
        print("\n⚠ ROS nodes are not running properly.")
        print("  Run: ros2 launch diff_drive_can debug_ps4_can.launch.py")
        return
    
    check_ros_topics()
    monitor_topics()
    monitor_can()
    
    print("\n" + "=" * 50)
    print("Test Summary:")
    print("If you see /joy and /cmd_vel messages but no CAN messages,")
    print("the issue is in the twist_to_can_node.")
    print("\nEnable debug mode with:")
    print("  ros2 param set /ps4_twist_node debug_mode true")
    print("  ros2 param set /twist_to_can_node debug_mode true")

if __name__ == "__main__":
    main()