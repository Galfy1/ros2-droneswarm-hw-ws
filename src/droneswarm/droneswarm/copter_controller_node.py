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
        self._current_ap_geopose = GeoPose()

        self._current_geopose_goal = GeoPose() # altitude in this variable is in agl
        self._start_altitude = 0.0 # Used for tracking altitude traversal. NOT to be confused with altitude of home location

        # This is something AP keeps track of. However, when using messages from the "/ap/goal_lla" topic, no target altitude is returned. Again, AP keeps track of this
        # but does not provide this information. So if we want to keep track of the distance to the desired altitude, it is something we have to keep track of manually. 
        self.home_geopose = GeoPose()


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
        # Updates the goal position based on the coordinate command provided and manually updates the traget altitude since this is
        # NOT provided through any topic.

        # stamp = msg.header.stamp
        # if stamp.sec:
        #     self._current_ap_goal.latitude = msg.position.latitude
        #     self._current_ap_goal.longitude = msg.position.longitude
            


    """ --- Methods --- """

    def _build_gps_pose_msg(self, lat: float, lon: float, agl_alt: float) -> GlobalPosition:
        """ 
        Builds geopose command message
        lat/lon - latetude and longitude of goal coordinates
        alt - altitude in meters above terrain (AGL). 
        """
        goal_pos = GlobalPosition()
        goal_pos.latitude = lat
        goal_pos.longitude = lon
        goal_pos.altitude = agl_alt # Absolute altitude above sea level AGL
        goal_pos.coordinate_frame = FRAME_GLOBAL_TERRAIN_ALT # Altitude in AGL
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

    def _update_goal_pose(self, lat: float, lon: float, agl_alt: float) -> None:
        
        # current_pose = self._current_ap_geopose
        # goal_a_alt = self._start_altitude + delta_alt

        if self._current_geopose_goal.position.latitude == lat and self._current_geopose_goal.position.longitude == lon and self._current_geopose_goal.position.altitude == agl_alt:
            return

        self._start_altitude = current_pose.position.altitude

        new_goal = GeoPose()
        new_goal.position.latitude = lat
        new_goal.position.longitude = lon
        new_goal.position.altitude = delta_alt
        self._current_geopose_goal = new_goal

        self.get_logger().info(
            "Goal updated to - Lat: {}, Lon: {}, Alt: {}".format(
                new_goal.position.latitude,
                new_goal.position.longitude,
                new_goal.position.altitude
            )
        )
    


    def cmd_navigate_to(self, lat, lon, agl_alt = 0.0) -> None:
        # Copter must be in GUIDED mode
        if self._current_ap_status_mode != COPTER_MODE_GUIDED:
            raise RuntimeError(
                f"Copter must be in GUIDED mode, but current mode is {self.current_ap_status_mode}"
            )

        current_alt = self._current_ap_geopose.position.altitude
        
        self._update_goal_pose(lat, lon, delta_alt)
        
        msg = self._build_gps_pose_msg(lat, lon, self._current_geopose_goal.position.altitude)
        self._global_pos_pub.publish(msg)

    def cmd_takeoff(self, delta_alt = 1.0) -> None:
        if self._current_ap_status_mode != COPTER_MODE_GUIDED:
            raise RuntimeError(
                f"Copter must be in GUIDED mode, but current mode is {self.current_ap_status_mode}"
            )
        if self._takeoff_in_progress:
            return

        lat = self._current_ap_geopose.position.latitude
        lon = self._current_ap_geopose.position.longitude
        alt = self._current_ap_geopose.position.altitude

        self._update_goal_pose(lat, lon, alt + delta_alt)

        self._takeoff_request(delta_alt)
        

    def has_reached_goal(self, tolerance = 1.0):
        # Copter must be in GUIDED mode
        if self._current_ap_status_mode != COPTER_MODE_GUIDED:
            raise RuntimeError(
                f"Copter must be in GUIDED mode, but current mode is {self._current_ap_status_mode}"
            )

        reached_goal = False

        current_goal_coor = (
            self._current_geopose_goal.position.latitude, 
            self._current_geopose_goal.position.longitude, 
            self._current_geopose_goal.position.altitude
            )
        current_pose_coor = (
            self._current_ap_geopose.position.latitude, 
            self._current_ap_geopose.position.longitude, 
            self._current_ap_geopose.position.altitude
            )

        print("goal: ", current_goal_coor[2])
        print("current: ", current_pose_coor[2])

        flat_distance = distance.distance(current_goal_coor[:2], current_pose_coor[:2]).m
        alt_distance = abs(current_goal_coor[2] - current_pose_coor[2])

        print("Flat distance: ", flat_distance, ", Altitude distance: ", alt_distance)

        # if reached_goal:
        #     self.delta_alt = 0.0

        # return reached_goal

        

    # def save_home_alt(self):
    #     self.home_geopose = self.current_geopose
    #     self.get_logger().info(
    #         "Set home geopose to - Lat: {}, Lon: {}, Alt: {}".format(
    #             self.home_geopose.position.latitude,
    #             self.home_geopose.position.longitude,
    #             self.home_geopose.position.altitude
    #         )
    #     )


    # def get_distance_to_waypoint(self):
    #     # Use 3D geopy distance calculation
    #     # https://geopy.readthedocs.io/en/stable/#module-geopy.distance

    #     current_goal = self._current_geopose_goal
    #     current_posistion = self._current_geopose.position

    #     p1 = (current_goal.latitude, current_goal.longitude, current_goal.altitude)
    #     p2 = (current_posistion.latitude, current_posistion.longitude, current_posistion.altitude)

    #     print("Current alt: ", p1[2], ", Goal alt: ", p2[2])

    #     flat_distance = distance.distance(p1[:2], p2[:2]).m
    #     alt_distance = abs(p2[2] - p1[2]) 
    #     print(f"Flat distance to goal: ", flat_distance, ", Altitude disantce to goal: ", alt_distance)

        

    """ --- Copter Control loop --- """

    def copter_control_loop(self):

        try:
            
            if not self._current_ap_status_flying:

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

                # Save Home altitude. We do this because no topic provide target altitude from takeoff geopose commands so its something we have to keep track of ourselves
                # self._save_home_alt()

                if not self._current_ap_status_flying or self._takeoff_in_progress:
                    self.cmd_takeoff(delta_alt = TAKEOFF_ALT)

            elif self._current_ap_status_flying:
                # self.cmd_navigate_to(
                #     lat = GRAYHOUND_TRACK.latitude, 
                #     lon = GRAYHOUND_TRACK.longitude, 
                #     alt = 20.0)

                self.has_reached_goal()
                    
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