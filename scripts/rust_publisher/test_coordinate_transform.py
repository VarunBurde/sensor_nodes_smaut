#!/usr/bin/env python3
"""
Test script to demonstrate the coordinate transformation logic
"""

def apply_gps_offset_demo(slam_x, slam_y, has_gps_ref=True, rtk_fix_status=2, current_offset_x=None, current_offset_y=None):
    """Demo version of the apply_gps_offset method"""
    
    def should_apply_gps_offset():
        return has_gps_ref and rtk_fix_status == 2
    
    if should_apply_gps_offset():
        # We need to store the SLAM position when GPS reference was first set
        if current_offset_x is None:
            # First time applying offset - store current SLAM position as offset
            offset_x = slam_x
            offset_y = slam_y
            print(f"🔄 Setting SLAM offset: x_offset={offset_x:.3f}, y_offset={offset_y:.3f}")
            print(f"   This means GPS reference point is at SLAM coordinates ({slam_x:.3f}, {slam_y:.3f})")
            
            # Apply offset so GPS reference point becomes (0,0) in local coordinates
            local_x = slam_x - offset_x  # This will be 0.0
            local_y = slam_y - offset_y  # This will be 0.0
            
            return local_x, local_y, offset_x, offset_y
        else:
            # Use existing offset
            local_x = slam_x - current_offset_x
            local_y = slam_y - current_offset_y
            return local_x, local_y, current_offset_x, current_offset_y
    else:
        # If no GPS reference or RTK fix, use original SLAM coordinates
        return slam_x, slam_y, current_offset_x, current_offset_y

def demo_coordinate_transformation():
    """Demonstrate how the coordinate transformation works"""
    print("🗺️  GPS Reference Point Coordinate Transformation Demo")
    print("=" * 60)
    
    # Simulate robot movement in SLAM coordinates
    slam_positions = [
        (10.5, 5.2),   # Robot at SLAM position when GPS ref is received
        (12.3, 6.1),   # Robot moves
        (8.7, 4.8),    # Robot moves back
        (15.0, 10.0),  # Robot moves far
        (10.5, 5.2)    # Robot returns to original GPS ref position
    ]
    
    print("Scenario: Robot receives GPS reference when at SLAM position (10.5, 5.2)")
    print("Goal: Make this position the new local map origin (0.0, 0.0)")
    print()
    
    offset_x, offset_y = None, None
    
    for i, (slam_x, slam_y) in enumerate(slam_positions):
        print(f"📍 Step {i+1}: Robot at SLAM coordinates ({slam_x:.1f}, {slam_y:.1f})")
        
        local_x, local_y, offset_x, offset_y = apply_gps_offset_demo(
            slam_x, slam_y, 
            has_gps_ref=True, 
            rtk_fix_status=2, 
            current_offset_x=offset_x, 
            current_offset_y=offset_y
        )
        
        print(f"   📡 Local coordinates: ({local_x:.1f}, {local_y:.1f})")
        
        if i == 0:
            print("   ✅ GPS reference established - this is now the local origin!")
        elif local_x == 0.0 and local_y == 0.0:
            print("   🎯 Robot is back at GPS reference point (local origin)!")
        
        print()
    
    print("📋 Summary:")
    print(f"   • GPS reference at SLAM coordinates: ({offset_x:.1f}, {offset_y:.1f})")
    print(f"   • Local map origin (0,0) corresponds to GPS reference point")
    print(f"   • All robot positions are now relative to GPS reference")

if __name__ == "__main__":
    demo_coordinate_transformation()
