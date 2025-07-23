#!/usr/bin/env python3

import zmq
import struct
import time
import socket

class SLAMZMQReader:
    def __init__(self, server_ip=None, port=27745):
        """
        Initialize ZMQ subscriber to read SLAM data
        
        Args:
            server_ip (str): IP address of the SLAM publisher. If None, uses local IP
            port (int): Port number to connect to
        """
        # If no server IP provided, use local IP
        if server_ip is None:
            server_ip = socket.gethostbyname(socket.gethostname())
        
        self.server_ip = server_ip
        self.port = port
        
        # ZMQ setup for subscribing to SLAM data
        self.zmq_context = zmq.Context()
        self.subscriber_socket = self.zmq_context.socket(zmq.SUB)
        self.endpoint = f"tcp://{server_ip}:{port}"
        
        # Connect to publisher
        self.subscriber_socket.connect(self.endpoint)
        self.subscriber_socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
        self.subscriber_socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
        
        print(f"ZMQ SLAM Reader initialized")
        print(f"Connected to: {self.endpoint}")
        print("Waiting for SLAM data...")
        print("Format: validity, robot_pose_x, robot_pose_y, robot_pose_phi")
        print("-" * 60)

    def read_slam_data(self):
        """Read and unpack SLAM data from ZMQ"""
        try:
            # Receive data from ZMQ
            data = self.subscriber_socket.recv(zmq.NOBLOCK)
            
            # Unpack data in the same format as handle_slam
            unpacked_data = struct.unpack(">Bddd", data)
            
            validity = bool(unpacked_data[0])
            robot_pose_x = unpacked_data[1]
            robot_pose_y = unpacked_data[2]
            robot_pose_phi = unpacked_data[3]
            
            return {
                "validity": validity,
                "robot_pose_x": robot_pose_x,
                "robot_pose_y": robot_pose_y,
                "robot_pose_phi": robot_pose_phi,
                "timestamp": time.time()
            }
            
        except zmq.Again:
            # No data available (timeout)
            return None
        except struct.error as e:
            print(f"Error unpacking SLAM data: {e}")
            return None
        except Exception as e:
            print(f"Error reading SLAM data: {e}")
            return None

    def run(self):
        """Main loop to continuously read and display SLAM data"""
        try:
            while True:
                slam_data = self.read_slam_data()
                
                if slam_data is not None:
                    # Display the received data
                    timestamp = time.strftime("%H:%M:%S", time.localtime(slam_data["timestamp"]))
                    validity_str = "VALID" if slam_data["validity"] else "INVALID"
                    
                    print(f"[{timestamp}] {validity_str} | "
                          f"X: {slam_data['robot_pose_x']:8.3f} | "
                          f"Y: {slam_data['robot_pose_y']:8.3f} | "
                          f"Phi: {slam_data['robot_pose_phi']:6.3f} rad")
                else:
                    # No data received, show waiting indicator
                    print(".", end="", flush=True)
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("\nSLAM ZMQ Reader interrupted by user")
        except Exception as e:
            print(f"\nError in main loop: {e}")
        finally:
            self.shutdown()

    def shutdown(self):
        """Clean shutdown"""
        print("\nShutting down SLAM ZMQ Reader")
        self.subscriber_socket.close()
        self.zmq_context.term()

def main():
    import argparse
    
    # Command line argument parsing
    parser = argparse.ArgumentParser(description='Read SLAM data from ZMQ publisher')
    parser.add_argument('--ip', type=str, default=None, 
                       help='IP address of SLAM publisher (default: local IP)')
    parser.add_argument('--port', type=int, default=27745,
                       help='Port number to connect to (default: 27745)')
    
    args = parser.parse_args()
    
    try:
        # Create and run the SLAM reader
        reader = SLAMZMQReader(server_ip=args.ip, port=args.port)
        reader.run()
        
    except Exception as e:
        print(f"Error starting SLAM ZMQ Reader: {e}")

if __name__ == '__main__':
    main()
