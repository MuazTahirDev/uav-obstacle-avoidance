#!/usr/bin/env python3

from Drone_API import *
import rospy
import math
import threading
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist

class ObstacleAvoidanceOnly:
    def __init__(self, drone):
        self.drone = drone
        
        # Avoidance parameters
        self.d0 = 3.0  # Detection radius (m)
        self.k = 2.0   # Repulsion gain
        self.climb_velocity = 1  # m/s
        self.vertical_threshold = 1  # m
        
        # Velocity limits
        self.max_avoid_speed = 1.5  # m/s - maximum avoidance velocity
        
        # State variables
        self.avoidance_vector_x = 0.0
        self.avoidance_vector_y = 0.0
        self.avoid_horizontal = False
        self.avoid_vertical = False
        self.min_obstacle_distance = 100.0
        
        # Thread lock
        self.lock = threading.Lock()
        
        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher("/drone1/mavros/setpoint_velocity/cmd_vel_unstamped", 
                                       Twist, queue_size=10)
        
        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/rangefinder/range", Range, self.range_callback)
        
        rospy.loginfo("Obstacle Avoidance Only - Initialized")
    
    def scan_callback(self, msg):
        """Process laser scan for horizontal obstacles"""
        with self.lock:
            avoidance_x = 0.0
            avoidance_y = 0.0
            avoid = False
            min_dist = 100.0
            
            for i, r in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                
                # Filter out invalid and very close readings (drone body)
                if r < 0.35 or math.isinf(r) or math.isnan(r):
                    continue
                
                if r < self.d0:
                    avoid = True
                    if r < min_dist:
                        min_dist = r
                    
                    # Unit vector pointing toward obstacle
                    x = math.cos(angle)
                    y = math.sin(angle)
                    
                    # Enhanced repulsion for close obstacles
                    if r < 1.0:
                        U = 5.0 * ((1.0 / r) - (1.0 / self.d0)) ** 2
                    elif r < 2.0:
                        U = 3.0 * ((1.0 / r) - (1.0 / self.d0)) ** 2
                    else:
                        U = self.k * ((1.0 / r) - (1.0 / self.d0)) ** 2
                    
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
        """Process rangefinder for vertical obstacles (ground)"""
        with self.lock:
            self.avoid_vertical = (msg.range < self.vertical_threshold)
    
    def is_avoiding(self):
        """Check if currently avoiding obstacles"""
        with self.lock:
            return self.avoid_horizontal or self.avoid_vertical
    
    def calculate_avoidance_velocity(self):
        """Calculate pure avoidance velocity"""
        with self.lock:
            vx = self.avoidance_vector_x
            vy = self.avoidance_vector_y
            vz = self.climb_velocity if self.avoid_vertical else 0.0
            min_dist = self.min_obstacle_distance
        
        # Limit horizontal velocity magnitude
        vel_mag = math.sqrt(vx**2 + vy**2)
        if vel_mag > self.max_avoid_speed:
            scale = self.max_avoid_speed / vel_mag
            vx *= scale
            vy *= scale
        
        return vx, vy, vz, min_dist
    
    def publish_velocity(self, vx, vy, vz):
        """Publish velocity command"""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        self.vel_pub.publish(cmd)


def run_avoidance_only():
    # Initialize the MAVROS_Drone object
    drone = MAVROS_Drone()
    
    # Initialize the ROS node
    drone.init_node()
    
    # Initialize publishers and subscribers
    drone.init_publishers()
    drone.init_subscribers()
    
    # Initialize avoidance
    avoider = ObstacleAvoidanceOnly(drone)

    # Wait for the system to initialize
    rospy.sleep(2)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("OBSTACLE AVOIDANCE ONLY MODE")
    rospy.loginfo("The drone will avoid obstacles while you control navigation")
    rospy.loginfo("Use RC or GCS to fly the drone - avoidance runs automatically")
    rospy.loginfo("=" * 60)
    
    rate = rospy.Rate(20)  # 20 Hz
    
    try:
        while not rospy.is_shutdown():
            # Calculate avoidance velocity
            vx, vy, vz, min_dist = avoider.calculate_avoidance_velocity()
            
            # Only publish if avoiding something
            if avoider.is_avoiding():
                avoider.publish_velocity(vx, vy, vz)
                
                rospy.loginfo_throttle(1.0, 
                    f"AVOIDING: Obstacle at {min_dist:.2f}m | "
                    f"Avoidance Vel: ({vx:.2f}, {vy:.2f}, {vz:.2f})")
            else:
                # No obstacles - publish zero velocity (no interference with manual control)
                avoider.publish_velocity(0, 0, 0)
                
                rospy.loginfo_throttle(5.0, "Path clear - monitoring...")
            
            rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("Avoidance system stopped.")
        # Stop all velocities
        avoider.publish_velocity(0, 0, 0)


if __name__ == "__main__":
    try:
        run_avoidance_only()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

