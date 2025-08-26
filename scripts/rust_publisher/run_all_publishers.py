#!/usr/bin/env python3
"""
Simple script to run both robot data publisher and radar publisher
"""

import subprocess
import sys
import time
import signal
import os

def signal_handler(sig, frame):
    print('\nShutting down publishers...')
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    print("Starting Robot Data Publishers...")
    print("Press Ctrl+C to stop all publishers")
    
    processes = []
    
    try:
        # Start robot telemetry publisher
        print("Starting robot telemetry publisher...")
        robot_data_proc = subprocess.Popen([
            sys.executable, 
            os.path.join(script_dir, "robot_telemetry_publisher.py")
        ])
        processes.append(robot_data_proc)
        time.sleep(2)
        
        # Start radar pointcloud publisher
        #print("Starting radar pointcloud publisher...")
        #radar_proc = subprocess.Popen([
        #    sys.executable, 
        #    os.path.join(script_dir, "radar_pointcloud_publisher.py")
        #])
        #processes.append(radar_proc)
        
        print("Both publishers started successfully!")
        print("\nPublished topics:")
        print("  - /robot/gps/fix (sensor_msgs/NavSatFix)")
        print("  - /robot/pose (geometry_msgs/PoseWithCovarianceStamped)")
        print("  - /robot/emergency_active (std_msgs/Bool)")
        print("  - /robot/autonomous_state (std_msgs/Bool)")
        print("  - /robot/wheel_angle (std_msgs/Float32)")
        print("  - /robot/gps/rtk_fix (std_msgs/Int32)")
        print("  - /robot/diagnostics (diagnostic_msgs/DiagnosticArray)")
        print("  - /radar/pointcloud (sensor_msgs/PointCloud2)")
        
        # Wait for processes to finish
        while True:
            time.sleep(1)
            # Check if any process has died
            for proc in processes:
                if proc.poll() is not None:
                    print(f"Process {proc.pid} has stopped")
                    
    except KeyboardInterrupt:
        print("\nShutting down...")
        
    finally:
        # Terminate all processes
        for proc in processes:
            if proc.poll() is None:
                print(f"Terminating process {proc.pid}")
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    print(f"Force killing process {proc.pid}")
                    proc.kill()

if __name__ == "__main__":
    main()
