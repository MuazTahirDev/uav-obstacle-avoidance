#!/usr/bin/env python3

from Drone_API import *
import rospy
import math
import threading
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist

class ObstacleAvoidanceHelper:
    def __init__(self, drone):
        self.drone = drone
        
        # Navigation speed limit
        self.max_nav_speed = 1.0  # Limit normal navigation to 2 m/s
        
        # Avoidance parameters
        self.d0 = 4.5  # Detection radius (m) - detect even earlier
        self.k = 4.0   # Very strong repulsion gain
        self.climb_velocity = 2.0  # m/s
        self.vertical_threshold = 1.0  # m
        self.safe_distance = 3  # Start avoiding at this distance
        
        # State variables
        self.avoidance_vector_x = 0.0
        self.avoidance_vector_y = 0.0
        self.avoid_horizontal = False
        self.avoid_vertical = False
        self.current_range = 100.0
        self.min_obstacle_distance = 100.0
        
        # Thread lock
        self.lock = threading.Lock()
        
        # Publisher for velocity commands (stays in GUIDED mode)
        self.vel_pub = rospy.Publisher("/drone1/mavros/setpoint_velocity/cmd_vel_unstamped", 
                                       Twist, queue_size=10)
        
        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/rangefinder/range", Range, self.range_callback)
        
        rospy.loginfo("Obstacle Avoidance Helper Initialized")
    
    def scan_callback(self, msg):
        """Process laser scan for horizontal obstacles"""
        with self.lock:
            avoidance_x = 0.0
            avoidance_y = 0.0
            avoid = False
            min_dist = 100.0
            
            for i, r in enumerate(msg.ranges):
                if 0.35 < r < self.d0:
                    avoid = True
                    min_dist = min(min_dist, r)
                    angle = msg.angle_min + i * msg.angle_increment
                    
                    # Unit vector pointing toward obstacle
                    x = math.cos(angle)
                    y = math.sin(angle)
                    
                    # Very strong repulsion force
                    U = 0.5 * self.k * ((1.0 / r) - (1.0 / self.d0)) ** 2
                    
                    # Much stronger when close
                    if r < 2.5:
                        U *= (3.0 / r) ** 2  # Quadratic scaling - very strong repulsion
                    
                    # Accumulate repulsion away from obstacle
                    avoidance_x += x * U
                    avoidance_y += y * U
            
            self.min_obstacle_distance = min_dist
            
            if avoid:
                self.avoid_horizontal = True
                self.avoidance_vector_x = avoidance_x
                self.avoidance_vector_y = avoidance_y
            else:
                self.avoid_horizontal = False
                self.avoidance_vector_x = 0.0
                self.avoidance_vector_y = 0.0
    
    def range_callback(self, msg):
        """Process rangefinder for vertical obstacles"""
        with self.lock:
            self.current_range = msg.range
            self.avoid_vertical = (msg.range < self.vertical_threshold)
    
    def is_avoiding(self):
        """Check if currently avoiding obstacles"""
        with self.lock:
            return self.avoid_horizontal or self.avoid_vertical
    
    def get_avoidance_velocity(self):
        """Get avoidance velocity in body frame"""
        with self.lock:
            vx = self.avoidance_vector_x
            vy = self.avoidance_vector_y
            vz = self.climb_velocity if self.avoid_vertical else 0.0
            
            # Limit horizontal speed but keep it strong
            mag = math.sqrt(vx**2 + vy**2)
            if mag > 4.0:  # Increased max speed for stronger avoidance
                vx = (vx / mag) * 4.0
                vy = (vy / mag) * 4.0
            
            return vx, vy, vz
    
    def publish_speed_limit(self):
        """Publish gentle velocity limit to slow down navigation"""
        cmd = Twist()
        cmd.linear.x = self.max_nav_speed
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        self.vel_pub.publish(cmd)


def move_drone_to_coordinates():
    # Initialize the MAVROS_Drone object
    drone = MAVROS_Drone()
    
    # Initialize the ROS node
    drone.init_node()
    
    # Initialize publishers and subscribers
    drone.init_publishers()
    drone.init_subscribers()
    
    # Initialize obstacle avoidance helper
    avoidance = ObstacleAvoidanceHelper(drone)

    # Wait for the system to initialize
    rospy.sleep(2)
    
    # Arm the drone with retries
    rospy.loginfo("Arming the drone...")
    for attempt in range(5):
        if drone.arm():
            rospy.loginfo("Drone armed successfully.")
            break
        else:
            rospy.logwarn(f"Arming attempt {attempt + 1} failed. Retrying...")
            rospy.sleep(2)
    else:
        rospy.logerr("Failed to arm the drone after multiple attempts. Aborting mission.")
        return

    # Set the mode to GUIDED
    rospy.loginfo("Setting mode to GUIDED...")
    for attempt in range(5):
        if drone.set_mode("GUIDED"):
            rospy.loginfo("Mode set to GUIDED.")
            break
        else:
            rospy.logwarn(f"Setting GUIDED mode attempt {attempt + 1} failed. Retrying...")
            rospy.sleep(2)
    else:
        rospy.logerr("Failed to set mode to GUIDED. Aborting mission.")
        return

    # Take off to 5 meters altitude with retries
    rospy.loginfo("Taking off to 2 meters...")
    for attempt in range(5):
        if drone.takeoff(altitude=2):
            rospy.loginfo("Takeoff command sent successfully.")
            break
        else:
            rospy.logwarn(f"Takeoff attempt {attempt + 1} failed. Retrying...")
            rospy.sleep(2)
    else:
        rospy.logerr("Failed to take off after multiple attempts. Aborting mission.")
        return

    # Wait until the drone has reached the takeoff altitude
    rospy.loginfo("Waiting for takeoff to complete...")
    while not drone.check_takeoff_complete():
        rospy.sleep(1)
    rospy.loginfo("Takeoff complete.")

    # Define the GPS coordinates to visit
    gps_coordinates = [
        (33.642402, 72.991180)
    ]

    # Move to each GPS coordinate with obstacle avoidance
    for latitude, longitude in gps_coordinates:
        rospy.loginfo(f"Moving to GPS coordinate: Latitude: {latitude}, Longitude: {longitude}, Altitude: 5m")
        
        # Send initial goto command
        drone.goto_location(latitude, longitude, altitude=2)

        # Navigate with obstacle avoidance
        rospy.loginfo("Navigating with obstacle avoidance enabled...")
        rate = rospy.Rate(10)  # 10 Hz control loop
        
        was_avoiding = False
        avoidance_counter = 0
        stuck_counter = 0
        last_distance = 999.0
        
        while True:
            current_lat = drone.data.global_position.latitude
            current_lon = drone.data.global_position.longitude
            distance = haversine_distance(current_lat, current_lon, latitude, longitude)
            
            # Check if obstacle detected
            is_avoiding = avoidance.is_avoiding()
            
            if is_avoiding:
                # Obstacle detected - send avoidance velocity
                vx, vy, vz = avoidance.get_avoidance_velocity()
                
                cmd = Twist()
                cmd.linear.x = vx
                cmd.linear.y = vy
                cmd.linear.z = vz
                
                avoidance.vel_pub.publish(cmd)
                
                if not was_avoiding:
                    rospy.logwarn(f"OBSTACLE DETECTED at {avoidance.min_obstacle_distance:.2f}m - Starting avoidance")
                    was_avoiding = True
                    avoidance_counter = 0
                
                avoidance_counter += 1
                
                # Check if stuck (obstacle distance not increasing)
                if avoidance.min_obstacle_distance < 3.0 and avoidance_counter > 20:
                    stuck_counter += 1
                else:
                    stuck_counter = 0
                
                # If stuck for 50 iterations (5 seconds), add lateral movement
                if stuck_counter > 50:
                    rospy.logerr("Stuck in avoidance! Adding lateral push to get around obstacle...")
                    
                    # Add a strong lateral component to push around the obstacle
                    lateral_cmd = Twist()
                    lateral_cmd.linear.x = vx * 1.5  # Stronger forward push
                    lateral_cmd.linear.y = vy * 1.5 if abs(vy) > 0.1 else 2.0  # Strong sideways
                    lateral_cmd.linear.z = 0.5  # Small climb component
                    
                    for _ in range(30):  # Push for 3 seconds
                        avoidance.vel_pub.publish(lateral_cmd)
                        rate.sleep()
                    
                    stuck_counter = 0
                    rospy.loginfo("Lateral push complete, resuming...")
                
                rospy.loginfo(f"Avoiding: dist={avoidance.min_obstacle_distance:.2f}m, target={distance:.2f}m")
                
            else:
                if was_avoiding:
                    # Just cleared obstacle
                    rospy.loginfo(f"Obstacle cleared! Resuming navigation to target ({distance:.2f}m remaining)")
                    
                    # Resume navigation at controlled speed
                    drone.goto_location(latitude, longitude, altitude=5)
                    
                    was_avoiding = False
                    stuck_counter = 0
                else:
                    # No obstacle - periodically refresh goto command
                    if avoidance_counter % 10 == 0:  # Every 1 second
                        drone.goto_location(latitude, longitude, altitude=5)
            
            # Check if reached target
            if distance < 2.0:
                rospy.loginfo("Reached target GPS coordinates.")
                break
            
            # Increment counter for periodic actions
            avoidance_counter += 1
            last_distance = distance
            rate.sleep()

    # Land the drone after reaching the final target
    rospy.loginfo("Landing the drone...")
    
    for attempt in range(5):
        if drone.set_mode("LAND"):
            rospy.loginfo("Drone is landing...")
            break
        else:
            rospy.logwarn(f"Landing attempt {attempt + 1} failed. Retrying...")
            rospy.sleep(2)
    else:
        rospy.logerr("Failed to initiate landing. Aborting mission.")
        return

    # Wait for landing to complete
    rospy.loginfo("Waiting for landing to complete...")
    while not drone.check_land_complete():
        rospy.sleep(1)
    rospy.loginfo("Landing complete.")

    # Disarm the drone
    rospy.loginfo("Disarming the drone...")
    if drone.disarm():
        rospy.loginfo("Drone disarmed successfully.")
    else:
        rospy.logerr("Failed to disarm the drone.")


def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great-circle distance between two points
    on the Earth's surface using the Haversine formula.
    """
    # Radius of Earth in meters
    R = 6378137.0

    # Convert latitude and longitude from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Differences in coordinates
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Haversine formula
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in meters
    distance = R * c
    return distance


if __name__ == "__main__":
    try:
        move_drone_to_coordinates()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
