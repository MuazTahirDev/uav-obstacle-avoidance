#!/usr/bin/python3

import math
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, ParamValue, State, RCIn
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamSet, StreamRate, CommandLong, CommandLongRequest
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

from Drone_Data import Drone_Data


class Custom_Drone():
    def __init__(self, ns=None) -> None:

        self.ns = ns if ns else ""
        self.data = Drone_Data()
        self.current_pose_g = Odometry()

        self.current_state_g = State()
        self.gps_data = None  # To store the current GPS coordinates
        self.correction_vector_g = Pose()
        self.local_offset_pose_g = Point()
        self.waypoint_g = PoseStamped()

        self.current_heading_g = 0.0
        self.local_offset_g = 0.0
        self.correction_heading_g = 0.0
        self.local_desired_heading_g = 0.0
        self.takeoff_altitude = 0.0

    def init_node(self):
        rospy.init_node("MAVROS_Drone_Node")           
        
    def init_subscribers(self):
        self.currentPos = rospy.Subscriber(
            name="{}/drone1/mavros/global_position/local".format(self.ns),
            data_class=Odometry,
            queue_size=10,
            callback=self.pose_cb,
        )

        self.state_sub = rospy.Subscriber(
            name="{}/drone1/mavros/state".format(self.ns),
            data_class=State,
            queue_size=10,
            callback=self.state_cb,
        )
        
        if self.ns:
            self.global_position_subscriber = rospy.Subscriber(self.ns + '/drone1/mavros/global_position/global',NavSatFix, self.global_sub_cb)
            self.local_position_subscriber = rospy.Subscriber(self.ns + '/drone1/mavros/global_position/local',Odometry, self.local_sub_cb)
            self.compass_hdg_subscriber = rospy.Subscriber(self.ns + '/drone1/mavros/global_position/compass_hdg',Float64, self.hdg_sub_cb)
            self.rel_alt_subscriber = rospy.Subscriber(self.ns + '/drone1/mavros/global_position/rel_alt',Float64, self.rel_alt_sub_cb)    
            self.state_subscriber = rospy.Subscriber(self.ns + '/drone1/mavros/state',State, self.state_sub_cb)
            self.rc_subscriber = rospy.Subscriber(self.ns + '/drone1/mavros/rc/in', RCIn, self.rc_sub_cb) 
            self.pose_subscriber = rospy.Subscriber(self.ns + '/drone1/mavros/local_position/odom', Odometry, self.pose_cb)
        else:
            self.global_position_subscriber = rospy.Subscriber('/drone1/mavros/global_position/global',NavSatFix, self.global_sub_cb)
            self.local_position_subscriber = rospy.Subscriber('/drone1/mavros/global_position/local',Odometry, self.local_sub_cb)
            self.compass_hdg_subscriber = rospy.Subscriber('/drone1/mavros/global_position/compass_hdg',Float64, self.hdg_sub_cb)
            self.rel_alt_subscriber = rospy.Subscriber('/drone1/mavros/global_position/rel_alt',Float64, self.rel_alt_sub_cb) 
            self.state_subscriber = rospy.Subscriber('/drone1/mavros/state',State, self.state_sub_cb) 
            self.rc_subscriber = rospy.Subscriber('/drone1/mavros/rc/in', RCIn, self.rc_sub_cb) 
            self.pose_subscriber = rospy.Subscriber('/drone1/mavros/local_position/odom', Odometry, self.pose_cb)
            
    def init_publishers(self):
        self.local_pos_pub = rospy.Publisher(
            name="{}/drone1/mavros/setpoint_position/local".format(self.ns),
            data_class=PoseStamped,
            queue_size=10,
        )    

        if self.ns:
            self.global_setpoint_publisher = rospy.Publisher(self.ns + '/drone1/mavros/setpoint_raw/global',GlobalPositionTarget, queue_size=1)
        else:
            self.global_setpoint_publisher = rospy.Publisher('/drone1/mavros/setpoint_raw/global',GlobalPositionTarget, queue_size=1)
            
        self.vel_pub = rospy.Publisher(
            name="{}/drone1/mavros/setpoint_velocity/cmd_vel".format(self.ns),
            data_class=TwistStamped,
            queue_size=10,
        )



    def check_GPS_fix(self):
        if self.data.global_position.gps_fix >= 0:
            return True
        else:
            return False

    def state_cb(self, message):
        self.current_state_g = message
        
    def set_velocity_body(self, vx, vy, vz, yaw_rate):
   
        vel_msg = TwistStamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = vz
        vel_msg.twist.angular.z = yaw_rate
        self.vel_pub.publish(vel_msg)

    
    def pose_cb(self, msg):
        """Gets the raw pose of the drone and processes it for use in control."""
        self.current_pose_g = msg
        self.enu_2_local()

        q0, q1, q2, q3 = (
            self.current_pose_g.pose.pose.orientation.w,
            self.current_pose_g.pose.pose.orientation.x,
            self.current_pose_g.pose.pose.orientation.y,
            self.current_pose_g.pose.pose.orientation.z,
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

        self.current_heading_g = degrees(psi) - self.local_offset_g

    def enu_2_local(self):
        x, y, z = (
            self.current_pose_g.pose.pose.position.x,
            self.current_pose_g.pose.pose.position.y,
            self.current_pose_g.pose.pose.position.z,
        )

        current_pos_local = Point()

        current_pos_local.x = x * cos(radians((self.local_offset_g - 90))) - y * sin(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.y = x * sin(radians((self.local_offset_g - 90))) + y * cos(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.z = z

        return current_pos_local

    def get_current_heading(self):
        """Returns the current heading of the drone."""
        return self.current_heading_g

    def get_current_location(self):
        """Returns the current position of the drone."""
        return self.enu_2_local()
    
    def arm(self):
        if self.ns:
            rospy.wait_for_service(self.ns + '/drone1/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy(self.ns + '/drone1/mavros/cmd/arming', CommandBool)
                armResponse = armService(True)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        else:
            rospy.wait_for_service('/drone1/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy('/drone1/mavros/cmd/arming', CommandBool)
                armResponse = armService(True)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        
        return armResponse

    def disarm(self):
        if self.ns:
            rospy.wait_for_service(self.ns + '/drone1/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy(self.ns + '/drone1/mavros/cmd/arming', CommandBool)
                armResponse = armService(False)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        else:
            rospy.wait_for_service('/drone1/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy('/drone1/mavros/cmd/arming', CommandBool)
                armResponse = armService(False)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        
        return armResponse
            
    def set_mode(self, mode):
        if self.ns:
            rospy.wait_for_service(self.ns + '/drone1/mavros/set_mode', timeout=3)
            try:
                modeService = rospy.ServiceProxy(self.ns + '/drone1/mavros/set_mode', SetMode)
                modeResponse = modeService(custom_mode=mode)
                rospy.loginfo(modeResponse)
            except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
        else:
            rospy.wait_for_service('/drone1/mavros/set_mode', timeout=3)
            try:
                modeService = rospy.ServiceProxy('/drone1/mavros/set_mode', SetMode)
                modeResponse = modeService(custom_mode=mode)
                rospy.loginfo(modeResponse)
            except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
        
        return modeResponse
            
    def takeoff(self, altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0):
        if self.ns is None:
            rospy.wait_for_service('/drone1/mavros/cmd/takeoff', timeout=3)
            try:
                takeoffService = rospy.ServiceProxy('/drone1/mavros/cmd/takeoff', CommandTOL)
                takeoffResponse = takeoffService(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
                rospy.loginfo(takeoffResponse)
            except rospy.ServiceException as e:
                print("Takeoff failed: %s" %e)
        else:
            rospy.wait_for_service(self.ns + '/drone1/mavros/cmd/takeoff', timeout=3)
            try:
                takeoffService = rospy.ServiceProxy(self.ns + '/drone1/mavros/cmd/takeoff', CommandTOL)
                takeoffResponse = takeoffService(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
                rospy.loginfo(takeoffResponse)
            except rospy.ServiceException as e:
                print("Takeoff failed: %s" %e)
            
        if takeoffResponse:
            self.takeoff_altitude = altitude
        else:
            self.takeoff_altitude = 0
            
        return takeoffResponse
    
    def set_stream_rate(self):
        if not self.ns:
            rospy.wait_for_service('/drone1/mavros/set_stream_rate', timeout=3)
            try:
                streamService = rospy.ServiceProxy('/drone1/mavros/set_stream_rate', StreamRate)
                streamService(0, 10, 1)
            except rospy.ServiceException as e:
                print("Setting Stream Rate failed: %s" %e)
        else:
            rospy.wait_for_service(self.ns + '/drone1/mavros/set_stream_rate', timeout=3)
            try:
                streamService = rospy.ServiceProxy(self.ns + '/drone1/mavros/set_stream_rate', StreamRate)
                streamService(0, 10, 1)
            except rospy.ServiceException as e:
                print("Setting Stream Rate failed: %s" %e)

    def wait_for_guided(self):
        while self.data.header.mode != 'GUIDED':
            time.sleep(0.5)
    
    def goto_location(self, latitude, longitude, altitude, type_mask=4088, coordinate_frame=6):
        command = GlobalPositionTarget()
        command.altitude = altitude
        command.latitude = latitude
        command.longitude = longitude
        command.type_mask=type_mask
        command.coordinate_frame=coordinate_frame
        
        self.global_setpoint_publisher.publish(command)
        
    def goto_location_heading(self, latitude, longitude, altitude, yaw, type_mask=4088, coordinate_frame=6):
        command = GlobalPositionTarget()
        command.altitude = altitude
        command.latitude = latitude
        command.longitude = longitude
        command.yaw = yaw
        command.type_mask=GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY | GlobalPositionTarget.IGNORE_VZ | GlobalPositionTarget.IGNORE_AFX | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ | GlobalPositionTarget.IGNORE_YAW_RATE
        command.coordinate_frame=coordinate_frame
        
        self.global_setpoint_publisher.publish(command)
    
    def set_param(self, param_name, param_value):
        if not self.ns:
            rospy.wait_for_service('/drone1/mavros/param/set', timeout=3)
            try:
                param_set = rospy.ServiceProxy('/drone1/mavros/param/set', ParamSet)
                
                value = ParamValue()
                value.integer = 0
                value.real = float(param_value)
                response = param_set(param_name, value)
                rospy.loginfo("{0} parameter set to {1}".format(param_name, param_value))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
        
        else:
            rospy.wait_for_service(self.ns + '/drone1/mavros/param/set', timeout=3)
            try:
                param_set = rospy.ServiceProxy(self.ns + '/drone1/mavros/param/set', ParamSet)
                value = ParamValue()
                value.integer = 0
                value.real = float(param_value)
                response = param_set(param_name, value)
                rospy.loginfo("{0} parameter set to {1}".format(param_name, param_value))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                
        return response
    
    def check_takeoff_complete(self):
        if abs(self.takeoff_altitude - self.data.local_position.z) <= 0.5:
            return True
        else:
            return False
            
    def land(self):
        rospy.wait_for_service('/drone1/mavros/set_mode')
        self.set_mode("LAND")
        rospy.loginfo("Landing initiated...")

        
    def check_land_complete(self):
        if self.data.local_position.z <= 0.5:
            return True
        else:
            return False
        
    def check_target_location_reached(self, location):
        pass
                 
    def global_sub_cb(self, mssg):
        self.data.global_position.gps_fix = mssg.status.status
        self.data.global_position.latitude = mssg.latitude
        self.data.global_position.longitude = mssg.longitude
        
    def local_sub_cb(self, mssg):
        self.data.local_position.x = mssg.pose.pose.position.x
        self.data.local_position.y = mssg.pose.pose.position.y
        self.data.local_position.z = mssg.pose.pose.position.z
        
    def hdg_sub_cb(self,mssg):
        self.data.euler_orientation.yaw = mssg.data

    def rel_alt_sub_cb(self, mssg):
        self.data.global_position.altitude = mssg.data
        
    def state_sub_cb(self, mssg):
        self.data.header.mode = mssg.mode

    def rc_sub_cb(self, mssg):
        self.data.rc = mssg.channels
        
    def offset_location(self, latitude, longitude, dNorth, dEast):
        earth_radius = 6378137.0 #Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*latitude/180))
        #New position in decimal degrees
        newlat = latitude + (dLat * 180/math.pi)
        newlon = longitude + (dLon * 180/math.pi)
        targetlocation = (newlat, newlon)
            
        return targetlocation
    
    def ellipsoid_offset_location(self, latitude, longitude, dNorth, dEast):
        # WGS84 Earth semi-major and semi-minor axes
        a = 6378137.0  # semi-major axis in meters
        b = 6356752.314245  # semi-minor axis in meters

        # Eccentricity squared
        e_squared = 1 - (b**2 / a**2)

        # Radius of curvature in the prime vertical
        N = a / math.sqrt(1 - e_squared * math.sin(math.radians(latitude))**2)

        # Latitude offset in radians
        dLat = dNorth / N

        # Longitude offset in radians
        dLon = dEast / (N * math.cos(math.radians(latitude)))

        # Update latitude and longitude in radians
        new_lat_radians = math.radians(latitude) + dLat
        new_lon_radians = math.radians(longitude) + dLon

        # Convert back to decimal degrees
        new_lat = math.degrees(new_lat_radians)
        new_lon = math.degrees(new_lon_radians)

        target_location = (new_lat, new_lon)
        
        return target_location
        
    # ADDED: Local frame navigation functions from old API
    def set_heading(self, heading):
        """Set drone heading in local reference frame"""
        self.local_desired_heading_g = heading
        heading = heading + self.correction_heading_g + self.local_offset_g

        rospy.loginfo("The desired heading is {}".format(self.local_desired_heading_g))

        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        self.waypoint_g.pose.orientation = Quaternion(qx, qy, qz, qw)

    def set_destination(self, x, y, z, psi):
        """Set waypoint destination in local reference frame"""
        self.set_heading(psi)

        theta = radians((self.correction_heading_g + self.local_offset_g - 90))

        Xlocal = x * cos(theta) - y * sin(theta)
        Ylocal = x * sin(theta) + y * cos(theta)
        Zlocal = z

        x = Xlocal + self.correction_vector_g.position.x + self.local_offset_pose_g.x
        y = Ylocal + self.correction_vector_g.position.y + self.local_offset_pose_g.y
        z = Zlocal + self.correction_vector_g.position.z + self.local_offset_pose_g.z

        rospy.loginfo("Destination set to x:{} y:{} z:{} origin frame".format(x, y, z))

        self.waypoint_g.pose.position = Point(x, y, z)
        self.local_pos_pub.publish(self.waypoint_g)
        
    def initialize_local_frame(self):
        """Initialize local reference frame based on current position"""
        self.local_offset_g = 0.0

        for i in range(30):
            rospy.sleep(0.1)

            q0, q1, q2, q3 = (
                self.current_pose_g.pose.pose.orientation.w,
                self.current_pose_g.pose.pose.orientation.x,
                self.current_pose_g.pose.pose.orientation.y,
                self.current_pose_g.pose.pose.orientation.z,
            )

            psi = atan2((2 * (q0 * q3 + q1 * q2)),
                        (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

            self.local_offset_g += degrees(psi)
            self.local_offset_pose_g.x += self.current_pose_g.pose.pose.position.x
            self.local_offset_pose_g.y += self.current_pose_g.pose.pose.position.y
            self.local_offset_pose_g.z += self.current_pose_g.pose.pose.position.z

        self.local_offset_pose_g.x /= 30.0
        self.local_offset_pose_g.y /= 30.0
        self.local_offset_pose_g.z /= 30.0
        self.local_offset_g /= 30.0

        rospy.loginfo("Coordinate offset set" )
        rospy.loginfo("The X-Axis is facing: {}".format(self.local_offset_g))
    
    def check_waypoint_reached(self, pos_tol=0.3, head_tol=0.01):
        """Check if waypoint is reached within tolerance"""
        self.local_pos_pub.publish(self.waypoint_g)

        dx = abs(self.waypoint_g.pose.position.x - self.current_pose_g.pose.pose.position.x)
        dy = abs(self.waypoint_g.pose.position.y - self.current_pose_g.pose.pose.position.y)
        dz = abs(self.waypoint_g.pose.position.z - self.current_pose_g.pose.pose.position.z)

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        cosErr = cos(radians(self.current_heading_g)) - cos(radians(self.local_desired_heading_g))
        sinErr = sin(radians(self.current_heading_g)) - sin(radians(self.local_desired_heading_g))

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))

        if dMag < pos_tol and dHead < head_tol:
            return 1
        else:
            return 0

    def drone_to_ned_conversion(self, dNorth, dEast, heading):
        if heading >= 270:
            hdg = heading - 270
            North = dNorth*math.sin(math.radians(hdg)) + dEast*math.cos(math.radians(hdg))
            East = -1*dNorth*math.cos(math.radians(hdg)) + dEast*math.sin(math.radians(hdg))
        elif heading <= 90:
            North = dNorth*math.cos(math.radians(heading)) - dEast*math.sin(math.radians(heading))
            East = dNorth*math.sin(math.radians(heading)) + dEast*math.cos(math.radians(heading))
        elif heading<=180 and heading>90:
            hdg = heading - 90
            North = -1*dNorth*math.sin(math.radians(hdg)) - dEast*math.cos(math.radians(hdg))
            East = dNorth*math.cos(math.radians(hdg)) - dEast*math.sin(math.radians(hdg))
        else:
            hdg = heading - 180
            North = -1*dNorth*math.cos(math.radians(hdg)) + dEast*math.sin(math.radians(hdg))
            East = -1*dNorth*math.sin(math.radians(hdg)) - dEast*math.cos(math.radians(hdg))
        return (North, East)
    
    def ned_to_drone_conversion(self, dNorth, dEast, heading):
        North = dNorth*math.cos(math.radians(heading)) - dEast*math.sin(math.radians(heading))
        East = dNorth*math.sin(math.radians(heading)) + dEast*math.cos(math.radians(heading))
        return (North, East)
