#!/usr/bin/env python3
"""
Test script to verify GPS reference point reception
This script mimics the GPS reference reading functionality
"""

import zmq
import struct
import time

CONVERT = 57.29577951308232  # Conversion factor from radians to degrees

# Define the IP and port as constants
IP_ADDRESS = "tcp://192.168.232.55:27746"

def test_gps_reference():
    """Test GPS reference point reception"""
    # Create a ZeroMQ context and subscriber socket
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(IP_ADDRESS)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics
    socket.setsockopt(zmq.RCVTIMEO, 5000)  # 5 second timeout

    print(f"Connecting to GPS reference at: {IP_ADDRESS}")
    print("Waiting for GPS reference point...")

    try:
        # Receive data from the socket
        data = socket.recv()
        # Unpack the received data
        lla_ref = struct.unpack('>ddd', data)
        
        lat_deg = CONVERT * lla_ref[0]
        lon_deg = CONVERT * lla_ref[1]
        alt = lla_ref[2]
        
        print(f"Received GPS reference point:")
        print(f"  Latitude: {lat_deg:.8f}°")
        print(f"  Longitude: {lon_deg:.8f}°")
        print(f"  Altitude: {alt:.2f}m")
        print(f"  Raw values: {lla_ref}")
        
        return True, (lat_deg, lon_deg, alt)
        
    except zmq.Again:
        print("Timeout: No GPS reference data received within 5 seconds")
        return False, None
    except struct.error as e:
        print(f"Failed to parse data: {e}")
        return False, None
    except Exception as e:
        print(f"Error: {e}")
        return False, None
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    success, ref_point = test_gps_reference()
    if success:
        print("\n✅ GPS reference test successful!")
        print(f"Reference point will be used as map origin (0,0)")
    else:
        print("\n❌ GPS reference test failed!")
        print("Check if the GPS reference publisher is running on tcp://192.168.232.55:27746")
