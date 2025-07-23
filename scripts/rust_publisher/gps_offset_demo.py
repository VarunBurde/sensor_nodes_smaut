#!/usr/bin/env python3
"""
Test script to demonstrate GPS offset functionality
"""

class GPSOffsetDemo:
    def __init__(self):
        self.gps_reference = {"is_set": False}
        self.rtk_fix_status = 0
        
    def set_gps_reference(self, lat, lon, alt):
        """Simulate setting GPS reference"""
        self.gps_reference = {
            "lat": lat,
            "lon": lon, 
            "alt": alt,
            "is_set": True
        }
        print(f"GPS reference set: Lat={lat:.8f}°, Lon={lon:.8f}°, Alt={alt:.2f}m")
        
    def set_rtk_fix(self, status):
        """Simulate RTK fix status"""
        self.rtk_fix_status = status
        print(f"RTK fix status: {status} {'(RTK FIXED!)' if status == 2 else ''}")
        
    def should_apply_gps_offset(self):
        """Check if we should apply GPS offset"""
        return self.gps_reference['is_set'] and self.rtk_fix_status == 2
        
    def apply_gps_offset(self, slam_x, slam_y):
        """Apply GPS reference offset to SLAM coordinates"""
        if self.should_apply_gps_offset():
            # Store the SLAM position when GPS reference was first set
            if not hasattr(self, '_slam_offset_x'):
                self._slam_offset_x = slam_x
                self._slam_offset_y = slam_y
                print(f"📍 Setting SLAM offset: x_offset={self._slam_offset_x:.3f}, y_offset={self._slam_offset_y:.3f}")
                print(f"🎯 This SLAM position ({slam_x:.3f}, {slam_y:.3f}) will now be the local map origin (0.0, 0.0)")
            
            # Apply offset so GPS reference point becomes (0,0) in local coordinates
            offset_x = slam_x - self._slam_offset_x
            offset_y = slam_y - self._slam_offset_y
            return offset_x, offset_y
        else:
            return slam_x, slam_y
            
    def simulate_slam_movement(self, slam_positions):
        """Simulate SLAM odometry updates"""
        print("\n🤖 Simulating robot movement with SLAM odometry:")
        print("=" * 60)
        
        for i, (x, y) in enumerate(slam_positions):
            local_x, local_y = self.apply_gps_offset(x, y)
            
            if self.should_apply_gps_offset():
                print(f"Step {i+1}: SLAM=({x:6.2f}, {y:6.2f}) -> Local=({local_x:6.2f}, {local_y:6.2f})")
            else:
                status = "No GPS ref" if not self.gps_reference['is_set'] else f"RTK={self.rtk_fix_status}"
                print(f"Step {i+1}: SLAM=({x:6.2f}, {y:6.2f}) -> Raw ({status})")

def main():
    demo = GPSOffsetDemo()
    
    print("🗺️  GPS Offset Demonstration")
    print("=" * 50)
    
    # Simulate some SLAM positions before GPS reference
    slam_positions = [
        (10.0, 5.0),   # Robot starts here
        (12.0, 7.0),   # Moves to here
        (15.0, 8.0),   # Then here (GPS reference will be set at this point)
        (18.0, 10.0),  # Then here
        (20.0, 12.0),  # Finally here
    ]
    
    print("\n1️⃣  Initial movement without GPS reference:")
    for i in range(2):
        x, y = slam_positions[i]
        local_x, local_y = demo.apply_gps_offset(x, y)
        print(f"   SLAM position: ({x}, {y}) -> Raw coordinates (no GPS ref)")
    
    print("\n2️⃣  GPS reference becomes available:")
    demo.set_gps_reference(50.12345678, 14.98765432, 250.5)
    
    print("\n3️⃣  Movement with GPS reference but no RTK fix:")
    x, y = slam_positions[2]
    local_x, local_y = demo.apply_gps_offset(x, y)
    print(f"   SLAM position: ({x}, {y}) -> Raw coordinates (RTK fix = {demo.rtk_fix_status})")
    
    print("\n4️⃣  RTK fix achieved:")
    demo.set_rtk_fix(2)
    
    print("\n5️⃣  Now all movement is relative to GPS reference point:")
    demo.simulate_slam_movement(slam_positions[2:])
    
    print("\n✅ Summary:")
    print(f"   • GPS reference point represents a real-world location")
    print(f"   • SLAM position (15.0, 8.0) was set as local map origin (0.0, 0.0)")
    print(f"   • All subsequent movements are relative to this point")
    print(f"   • Your local map now has (0,0) at the GPS reference location!")

if __name__ == "__main__":
    main()
