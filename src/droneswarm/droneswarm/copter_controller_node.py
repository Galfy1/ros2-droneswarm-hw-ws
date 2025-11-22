#!/usr/bin/env python3

import rclpy
import time
import math
from geopy import point
from geopy import distance # https://github.com/geopy/geopy/blob/master/geopy/distance.py

from rclpy.node import Node
from std_srvs.srv import Trigger                # https://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html
from ardupilot_msgs.srv import ArmMotors        # https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/ardupilot_msgs/srv/ArmMotors.srv
from ardupilot_msgs.srv import ModeSwitch       # https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/ardupilot_msgs/srv/ModeSwitch.srv
from ardupilot_msgs.srv import Takeoff          # https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/ardupilot_msgs/srv/Takeoff.srv
from ardupilot_msgs.msg import Status           # https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/ardupilot_msgs/msg/Status.msg
from ardupilot_msgs.msg import GlobalPosition
from geographic_msgs.msg import GeoPointStamped # https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/Idl/geographic_msgs/msg/GeoPointStamped.idl
from geographic_msgs.msg import GeoPoint        # https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/Idl/geographic_msgs/msg/GeoPoint.idl 
from geographic_msgs.msg import GeoPoseStamped  # https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/Idl/geographic_msgs/msg/GeoPoseStamped.idl
from geographic_msgs.msg import GeoPose         # https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/Idl/geographic_msgs/msg/GeoPose.idl 


# https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE
COPTER_MODE_STABILIZE = 0
COPTER_MODE_GUIDED = 4

FRAME_GLOBAL_INT = 5
FRAME_GLOBAL_REL_ALT = 6
FRAME_GLOBAL_TERRAIN_ALT = 11

TAKEOFF_ALT = 5.0

# Hard code some known locations
# Note - Altitude in geopy is in km!
GRAYHOUND_TRACK = point.Point(latitude=-35.345996, longitude=149.159017, altitude=0.584)
# CMAC = point.Point(latitude=-35.3627010, longitude=149.1651513, altitude=0.585)

#temp
STATE1 = 1
STATE2 = 2

class CopterControllerNode(Node):

    def __init__(self):
        """ Initialize the copter controller node """
        super().__init__("copter_controller")

    
        # --- Create callback timers --- 
        self._copter_controll_loop_timer = self.create_timer(0.1, self.copter_control_loop)  # 10 Hz
        self._prearm_check_timer = self.create_timer(2.0, self._prearm_check)  # .5Hz

        # --- Services ---
        self._client_prearm = self.create_client(Trigger, "/ap/prearm_check")
        self._client_arm = self.create_client(ArmMotors, "/ap/arm_motors")
        self._client_mode_switch = self.create_client(ModeSwitch, "/ap/mode_switch")
        self._client_takeoff = self.create_client(Takeoff, "/ap/experimental/takeoff")

        while not self._client_prearm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('prearm service not available, waiting again...')

        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')

        while not self._client_takeoff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')


        # Configure QoS profile for subscribing
        qos_status_geopose = rclpy.qos.QoSProfile(
            reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
            durability = rclpy.qos.DurabilityPolicy.VOLATILE,
            depth = 1
        )

        # qos_geopoint = rclpy.qos.QoSProfile(
        #     reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        #     durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        #     depth=1,
        # )


        # --- Topics ---
        # Subscribers
        self._ap_status_sub = self.create_subscription(Status, "/ap/status", self._ap_status_callback, qos_status_geopose)
        self._ap_geopose_sub = self.create_subscription(GeoPoseStamped, "/ap/geopose/filtered", self._ap_geopose_callback, qos_status_geopose) # This is the current estimated position of the vehicle, coming from ArduPilot’s EKF.
        # self._ap_geopoint_sub = self.create_subscription(GeoPointStamped, "/ap/goal_lla", self._ap_geopoint_callback, qos_geopoint) # The goal (target) position that ArduPilot believes it is supposed to fly toward in GUIDED mode

        # Publishers
        self._global_pos_pub = self.create_publisher(GlobalPosition, "/ap/cmd_gps_pose", 1)


        # --- Control flags and variables ---
        # Initializing copter controller flags
        self._prearm_check_in_progress = False
        self._is_prearm_ok = False
        self._arming_in_progress = False
        self._mode_switch_in_progress = False
        self._takeoff_in_progress = False
        self._is_takeoff_ok = False

        # Updated by _ap_status_callback() from /ap/status topic
        self._current_ap_status_mode = COPTER_MODE_STABILIZE # Copter starts in stabilized mode.
        self._current_ap_status_armed = False
        self._current_ap_status_flying = False
        
        # Updated by _ap_geopose_callback() from "/ap/geopose/filtered"
        self._current_ap_geopose = GeoPose() # NOTE: Altitude in AMSL

        self._current_geopose_goal = GeoPose() # NOTE: Altitude is saved in the OSD format - relative altitude above origin/home
        self._origin_altitude = None

        # temp
        self.state = STATE1


    """ --- Service Methods  --- """

    def _prearm_check(self) -> None:
        if self._prearm_check_in_progress:
            return

        self._prearm_check_in_progress = True

        req = Trigger.Request()
        future = self._client_prearm.call_async(req)
        future.add_done_callback(self._on_prearm_check_complete)

    def _on_prearm_check_complete(self, future) -> None:

        # If we already passed prearm, ignore late results
        if self._prearm_check_timer is None or self._is_prearm_ok:
            return
        
        try:
            self._is_prearm_ok = future.result().success
        except Exception:
            self._is_prearm_ok = False

        self._prearm_check_in_progress = False

        # Stop checking if success
        if self._is_prearm_ok:
            self.get_logger().info("Prearm OK. Stopping prearm checks.")
            self._prearm_check_timer.cancel()
            self._prearm_check_timer = None
            return

        self.get_logger().warn(f"Prearm check failed - AP: {future.result().message}")


    def _arm_request(self) -> None:
        if self._arming_in_progress or self._current_ap_status_armed:
            return

        self.get_logger().info("Starting arming attempt...")
        self._arming_in_progress = True

        req = ArmMotors.Request()
        req.arm = True # True to arm, False to disarm
        future = self._client_arm.call_async(req)
        future.add_done_callback(self._on_arm_complete)

    def _on_arm_complete(self, future) -> None:

        try:
            armed = future.result().result
        except Exception:
            armed = False

        self._arming_in_progress = False

        if armed:
            self.get_logger().info("Arming OK!")
            return

        self.get_logger().warn("Arming failed.")


    def _switch_mode_request(self, mode) -> None:
        if self._mode_switch_in_progress:
            return

        self.get_logger().info(f"Starting mode switch attempt to mode {mode}...")
        self._mode_switch_in_progress = True

        req = ModeSwitch.Request()
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        future.add_done_callback(self._on_mode_switch_complete)

    def _on_mode_switch_complete(self, future) -> None:

        try:
            is_mode_ok = future.result().status
        except Exception:
            is_mode_ok = False

        self._mode_switch_in_progress = False

        if is_mode_ok:
            self.get_logger().info("Mode switch OK!")
            return

        self.get_logger().warn("Mode switch failed.")


    def _takeoff_request(self, alt: float) -> None:
        if self._current_ap_status_flying or self._takeoff_in_progress:
            return
        
        self.get_logger().info(f"Requesting takeoff to {alt} meters.")
        self._takeoff_in_progress = True

        req = Takeoff.Request()
        req.alt = alt
        future = self._client_takeoff.call_async(req)
        future.add_done_callback(self._on_takeoff_complete)
        
    def _on_takeoff_complete(self, future) -> None:
        """
        This callback does NOT mean takeoff is complete in real world,
        just that ardupilot accepted/rejected the service request.
        """
        try:
            takeoff_status = future.result().status
        except Exception:
            takeoff_status = False

        # Allow new attempts if takeoff failed
        if not takeoff_status:
            self.get_logger().warn("Takeoff request failed.")
            self._takeoff_in_progress = False
            return

        # If request succeeded, leave takeoff_in_progress = True
        # until FC reports that we are flying.
        # TODO: add later


    """ --- Callbacks --- """

    def _ap_status_callback(self, msg: Status) -> None:
        stamp = msg.header.stamp
        if stamp.sec:
            self._current_ap_status_mode = msg.mode
            self._current_ap_status_armed = msg.armed
            self._current_ap_status_flying = msg.flying

    def _ap_geopose_callback(self, msg: GeoPoseStamped) -> None:
        stamp = msg.header.stamp
        if stamp.sec:
            self._current_ap_geopose = msg.pose

    # def _ap_geopoint_callback(self, msg: GeoPointStamped) -> None:
    #     # Updates the goal position based on the coordinate command provided and manually updates the traget altitude since this is
    #     # NOT provided through any topic.

    #     stamp = msg.header.stamp
    #     if stamp.sec:
    #         self._current_ap_goal.latitude = msg.position.latitude
    #         self._current_ap_goal.longitude = msg.position.longitude
            


    """ --- Methods --- """

    def _build_gps_pose_msg(self, lat: float, lon: float, alt_osd: float) -> GlobalPosition:
        """ 
        Builds geopose command message
        lat/lon - latetude and longitude of goal coordinates
        alt - altitude in meters above origin (OSD). 
        """
        
        goal_pos = GlobalPosition()
        goal_pos.latitude = lat
        goal_pos.longitude = lon
        goal_pos.altitude = alt_osd # Altitude relative to origin location
        goal_pos.coordinate_frame = FRAME_GLOBAL_REL_ALT 
        goal_pos.header.frame_id = "map"

        goal_pos.type_mask = (
            GlobalPosition.IGNORE_VX |
            GlobalPosition.IGNORE_VY |
            GlobalPosition.IGNORE_VZ |
            GlobalPosition.IGNORE_AFX |
            GlobalPosition.IGNORE_AFY |
            GlobalPosition.IGNORE_AFZ |
            GlobalPosition.IGNORE_YAW |
            GlobalPosition.IGNORE_YAW_RATE
                    )

        return goal_pos

    def set_origin_alt(self):
        if self._origin_altitude is not None:
            return 

        self._origin_altitude = self._current_ap_geopose.position.altitude
        self.get_logger().info(f"Set origin altitude to: {self._origin_altitude}")

    def _update_goal_pose(self, lat: float, lon: float, alt_osd: float) -> None:
    

        if self._current_geopose_goal.position.latitude == lat and self._current_geopose_goal.position.longitude == lon and self._current_geopose_goal.position.altitude == alt_osd:
            return

        new_goal = GeoPose()
        new_goal.position.latitude = lat
        new_goal.position.longitude = lon
        new_goal.position.altitude = alt_osd
        self._current_geopose_goal = new_goal

        self.get_logger().info(
            "Goal updated to - Lat: {}, Lon: {}, Alt: {}".format(
                new_goal.position.latitude,
                new_goal.position.longitude,
                new_goal.position.altitude
            )
        )
    

    def cmd_navigate_to(self, lat, lon, alt_osd) -> None:
        # Copter must be in GUIDED mode
        if self._current_ap_status_mode != COPTER_MODE_GUIDED:
            raise RuntimeError(
                f"Copter must be in GUIDED mode, but current mode is {self.current_ap_status_mode}"
            )
        
        self._update_goal_pose(lat, lon, alt_osd)
        
        msg = self._build_gps_pose_msg(lat, lon, alt_osd)
        self._global_pos_pub.publish(msg)

    def cmd_takeoff(self, alt_osd = 1.0) -> None:
        if self._current_ap_status_mode != COPTER_MODE_GUIDED:
            raise RuntimeError(
                f"Copter must be in GUIDED mode, but current mode is {self.current_ap_status_mode}"
            )
        if self._takeoff_in_progress:
            return

        lat = self._current_ap_geopose.position.latitude
        lon = self._current_ap_geopose.position.longitude
        alt = self._current_ap_geopose.position.altitude

        self._update_goal_pose(lat, lon, alt_osd)

        self._takeoff_request(alt_osd)
        
    def has_reached_goal(self, dist_tolerance = 1.0, alt_tolerance = 1.0) -> bool:
        # Copter must be in GUIDED mode
        if self._current_ap_status_mode != COPTER_MODE_GUIDED:
            raise RuntimeError(
                f"Copter must be in GUIDED mode, but current mode is {self._current_ap_status_mode}"
            )

        current_goal_coor = (
            self._current_geopose_goal.position.latitude, 
            self._current_geopose_goal.position.longitude, 
            self._current_geopose_goal.position.altitude # OSD ALT
            )
        current_pose_coor = (
            self._current_ap_geopose.position.latitude, 
            self._current_ap_geopose.position.longitude, 
            self._current_ap_geopose.position.altitude - self._origin_altitude # AMSL ALT - ORIGIN ALT = OSD ALT
            )

        flat_distance = distance.distance(current_goal_coor[:2], current_pose_coor[:2]).m
        alt_distance = abs(current_goal_coor[2] - current_pose_coor[2])

        print("Flat distance: ", flat_distance, ", Altitude distance: ", alt_distance)

        if flat_distance < dist_tolerance and alt_distance < alt_tolerance:
            return True
        else:
            return False
        

    """ --- Copter Control loop --- """

    def copter_control_loop(self):

        try:
            
            if self.state == STATE1:
                if not self._is_prearm_ok:
                    return

                if not self._mode_switch_in_progress and self._current_ap_status_mode != COPTER_MODE_GUIDED:
                    self._switch_mode_request(COPTER_MODE_GUIDED)
                    return
                elif self._mode_switch_in_progress:
                    return

                # Start arming once prearm succeeds and mode switch completed
                if not self._arming_in_progress and not self._current_ap_status_armed:
                    self._arm_request()
                    return
                elif self._arming_in_progress:
                    return

                if not self._current_ap_status_flying and not self._takeoff_in_progress:

                    # Save origin altitude. We do this because no topic provide target altitude from takeoff geopose commands so its something we have to keep track of ourselves
                    self.set_origin_alt()

                    self.cmd_takeoff(alt_osd = TAKEOFF_ALT)

                if self.has_reached_goal():
                    self.state = STATE2


            elif self.state == STATE2:
                
                self.cmd_navigate_to(
                    lat = GRAYHOUND_TRACK.latitude, 
                    lon = GRAYHOUND_TRACK.longitude, 
                    alt_osd = 20.0)

                if self.has_reached_goal():
                    print("GOAL REACHED")
                


            # if not self._current_ap_status_flying:

            #     if not self._is_prearm_ok:
            #         return

            #     if not self._mode_switch_in_progress and self._current_ap_status_mode != COPTER_MODE_GUIDED:
            #         self._switch_mode_request(COPTER_MODE_GUIDED)
            #         return
            #     elif self._mode_switch_in_progress:
            #         return

            #     # Start arming once prearm succeeds and mode switch completed
            #     if not self._arming_in_progress and not self._current_ap_status_armed:
            #         self._arm_request()
            #         return
            #     elif self._arming_in_progress:
            #         return

            #     if not self._current_ap_status_flying or self._takeoff_in_progress:

            #         # Save origin altitude. We do this because no topic provide target altitude from takeoff geopose commands so its something we have to keep track of ourselves
            #         self.set_origin_alt()

            #         self.cmd_takeoff(alt_osd = TAKEOFF_ALT)

            # elif self._current_ap_status_flying:

            #     if self.has_reached_goal()

            #     self.cmd_navigate_to(
            #         lat = GRAYHOUND_TRACK.latitude, 
            #         lon = GRAYHOUND_TRACK.longitude, 
            #         alt = 20.0)

                
                    
        except Exception as e:
            self.get_logger().error(f"Copter control loop error: {e}")


# NOTE: Altitude i _current_geopose_goal er ikke altitude for dens goal, men altitude ved dens HOME lokalitet. Derfor skal der ligges altitude oven i home location for at få den absolutte højde over havet
# TODO: Der skal derfor laves lidt om i variablen der holder goal koordinatorne. Højden skal beregnes ved siden af 
    


def main(args=None):
    """ Copter control node entry point."""
    print("Starting copter control loop.")
    rclpy.init(args=args)
    
    node = CopterControllerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)