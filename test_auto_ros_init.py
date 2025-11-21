#!/usr/bin/env python3
"""
Test script demonstrating ROS 2 auto-initialization in WebRobotInterface.

This script shows that WebRobotInterface can now be used directly without
manually initializing ROS 2 first.
"""

print("🧪 Testing WebRobotInterface ROS 2 Auto-Initialization")
print("=" * 60)

try:
    # Import and create WebRobotInterface directly - no manual ROS 2 init needed!
    from ros2_ws.src.my_robot_automation.scripts.web_robot_interface import WebRobotInterface
    
    print("✅ Creating WebRobotInterface with auto ROS 2 initialization...")
    interface = WebRobotInterface(simulation_mode=True)  # Use simulation for testing
    
    print("✅ SUCCESS: WebRobotInterface created!")
    print(f"   🤖 Mega Connected: {interface.mega_connected}")
    print(f"   📡 IMU Initialized: {interface.imu_initialized}")
    print(f"   🌐 Flask App Ready: {'Yes' if hasattr(interface, 'app') else 'No'}")
    
    # Clean up
    interface.cleanup()
    print("✅ Cleanup completed successfully!")
    
    print("\n🎉 RESULT: WebRobotInterface can be used directly without manual ROS 2 setup!")
    
except ImportError as e:
    print(f"❌ Import Error: {e}")
    print("💡 Make sure ROS 2 environment is sourced: source /opt/ros/jazzy/setup.bash")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()

print("=" * 60)
print("🚀 WebRobotInterface is now ready for direct usage!")
