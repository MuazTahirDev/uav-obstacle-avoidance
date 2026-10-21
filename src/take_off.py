#!/usr/bin/env python
import rospy
from custom_API import *

def main():
    drone = Custom_Drone()

    # Initialize ROS node and publishers
    drone.init_node()
    drone.init_publishers()
    drone.init_subscribers()

    rospy.sleep(2)
    print("Starting mission...")

    # Arm and set mode
    drone.arm()
    drone.set_mode("GUIDED")
    drone.initialize_local_frame()

    # Takeoff to 5 meters
    target_alt = 2.0
    print(f"[TAKEOFF] Ascending to {target_alt}m...")
    drone.takeoff(target_alt)

  
    rospy.sleep(5)
    print("[SUCCESS] Mission complete!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("[INFO] Mission interrupted")
    except Exception as e:
        print(f"[ERROR] Mission failed: {e}")
        try:
            drone = Custom_Drone()
            drone.init_publishers()
            drone.land()
        except:
            pass
