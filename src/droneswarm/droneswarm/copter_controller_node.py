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
from ardupilot_msgs.msg import GlobalPosition   # https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/ardupilot_msgs/msg/GlobalPosition.msg
from geographic_msgs.msg import GeoPoseStamped  # https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/Idl/geographic_msgs/msg/GeoPoseStamped.idl
from geographic_msgs.msg import GeoPose         # https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/Idl/geographic_msgs/msg/GeoPose.idl 
from geometry_msgs.msg import TwistStamped      # https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_DDS/Idl/geometry_msgs/msg/TwistStamped.idl
from our_custom_interfaces.msg import ObjectData

from droneswarm.copter_mission_logic import CopterControllerFSM
from droneswarm.utility.ema_filter import EMAFilter
import droneswarm.utility.constants as const
import droneswarm.utility.settings as setting

class CopterControllerNode(Node):

    def __init__(self):
        """ Initialize the copter controller node """
        super().__init__("copter_controller")

        # --- Create callback timers --- 
        self._copter_controll_loop_timer = self.create_timer(setting.COPTER_CONTROL_LOOP_DT, self._copter_control_loop) 
        self._prearm_check_timer = self.create_timer(setting.PREARM_LOOP_DT, self._prearm_check) 
        self._vel_stream_loop_timer = None  

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

        # --- Topics ---
        # Subscribers
        self._ap_status_sub = self.create_subscription(Status, "/ap/status", self._ap_status_callback, qos_status_geopose)
        self._ap_geopose_sub = self.create_subscription(GeoPoseStamped, "/ap/geopose/filtered", self._ap_geopose_callback, qos_status_geopose) # This is the current estimated position of the vehicle, coming from ArduPilot’s EKF.
        self._detection_sub = self.create_subscription(ObjectData, "/detections", self._detections_callback, qos_status_geopose)

        # Publishers
        self._global_pos_pub = self.create_publisher(GlobalPosition, "/ap/cmd_gps_pose", 1)
        self._vel_pub = self.create_publisher(TwistStamped, "/ap/cmd_vel", 1)

        # --- Mission Logic FSM ---
        self._controller_fsm = CopterControllerFSM(node = self)

        # --- Control flags and variables ---
        # Initializing copter controller flags
        self.prearm_check_in_progress = False
        self.is_prearm_ok = False
        self.arming_in_progress = False
        self.mode_switch_in_progress = False
        self.takeoff_in_progress = False

        # Updated by _ap_status_callback() from /ap/status topic
        self.current_ap_status_mode = const.COPTER_MODE_STABILIZE # Copter starts in stabilized mode.
        self.current_ap_status_armed = False
        self.current_ap_status_flying = False
        
        # Internal mode 
        self.internal_mode = const.INTERNAL_MODE_STABILIZED

        # Updated by _ap_geopose_callback() from "/ap/geopose/filtered"
        self._current_ap_geopose = GeoPose() # NOTE: Altitude in AMSL

        self._current_geopose_goal = GeoPose() # NOTE: Altitude is saved in the OSD format - relative altitude above origin/home
        self._origin_altitude = None

        # Linear and Angular velocity commands
        self._desired_linear_vel = (0.0, 0.0, 0.0) # x, y, z
        self._desired_angular_vel = (0.0, 0.0, 0.0) # Roll, Pitch, Yaw
        self._vel_watchdog_counter = 0

        # Dectecions error filters
        self.err_x_filter = EMAFilter(alpha = setting.ERR_X_EMA_APLHA)    # Pixel error between the bounding box center and the image center along the x-axis.
        self.err_y_filter = EMAFilter(alpha = setting.ERR_Y_EMA_APLHA)    # Pixel error between the bounding box center and the image center along the y-axis.
        self.w_filter = EMAFilter(alpha = setting.BBOX_W_EMA_APLHA)       # Pixel size of bounding box width edge
        self.h_filter = EMAFilter(alpha = setting.BBOX_H_EMA_APLHA)       # Pixel size of bounding box height edge

        # Newest detection data from PI OS node - Updated by the _detections_callback()
        self.have_detection = False
        self.filtered_err_x = 0.0
        self.filtered_err_y = 0.0
        self.filtered_bbox_w = 0.0
        self.filtered_bbox_h = 0.0
        self.last_detection_time = time.perf_counter()
        

    """ --- Service Methods  --- """

    def _prearm_check(self) -> None:
        if self.prearm_check_in_progress:
            return

        self.prearm_check_in_progress = True

        req = Trigger.Request()
        future = self._client_prearm.call_async(req)
        future.add_done_callback(self._on_prearm_check_complete)

    def _on_prearm_check_complete(self, future) -> None:

        # If we already passed prearm, ignore late results
        if self._prearm_check_timer is None or self.is_prearm_ok:
            return
        
        try:
            self.is_prearm_ok = future.result().success
        except Exception:
            self.is_prearm_ok = False

        self.prearm_check_in_progress = False

        # Stop checking if success
        if self.is_prearm_ok:
            self.get_logger().info("Prearm OK. Stopping prearm checks.")
            self._prearm_check_timer.cancel()
            self._prearm_check_timer = None
            return

        self.get_logger().warn(f"Prearm check failed - AP: {future.result().message}")


    def _arm_request(self) -> None:

        self.get_logger().info("Starting arming attempt...")
        self.arming_in_progress = True

        req = ArmMotors.Request()
        req.arm = True # True to arm, False to disarm
        future = self._client_arm.call_async(req)
        future.add_done_callback(self._on_arm_complete)

    def _on_arm_complete(self, future) -> None:

        try:
            armed = future.result().result
        except Exception:
            armed = False

        self.arming_in_progress = False

        if armed:
            self.get_logger().info("Arming OK!")
            return

        self.get_logger().warn("Arming failed.")


    def _switch_mode_request(self, mode: int) -> None:
        self.get_logger().info(f"Starting mode switch attempt to mode {mode}...")

        req = ModeSwitch.Request()
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        future.add_done_callback(self._on_mode_switch_complete)

    def _on_mode_switch_complete(self, future) -> None:

        try:
            is_mode_ok = future.result().status
        except Exception:
            is_mode_ok = False

        self.mode_switch_in_progress = False

        if is_mode_ok:
            self.get_logger().info("Mode switch OK!")
            return

        self.get_logger().warn("Mode switch failed.")


    def _takeoff_request(self, alt: float) -> None:
        if self.current_ap_status_flying or self.takeoff_in_progress:
            return
        
        self.get_logger().info(f"Requesting takeoff to {alt} meters.")
        self.takeoff_in_progress = True

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
            self.takeoff_in_progress = False
            return

        # If request succeeded, leave takeoff_in_progress = True
        # until FC reports that we are flying.
        # TODO: add later


    """ --- Topic callbacks --- """

    def _ap_status_callback(self, msg: Status) -> None:
        stamp = msg.header.stamp
        if stamp.sec:
            self.current_ap_status_mode = msg.mode
            self.current_ap_status_armed = msg.armed
            self.current_ap_status_flying = msg.flying

    def _ap_geopose_callback(self, msg: GeoPoseStamped) -> None:
        stamp = msg.header.stamp
        if stamp.sec:
            self._current_ap_geopose = msg.pose

    def _detections_callback(self, msg: ObjectData) -> None:

        self.have_detection = msg.valid
        self.last_detection_time = time.perf_counter() # Used for grace period 

        self.filtered_err_x = self.err_x_filter.update(-msg.err_x)  # Inverting the sign since the controls for controlling the drone is flipped
        self.filtered_err_y = self.err_y_filter.update(-msg.err_y)  # Inverting the sign since the controls for controlling the drone is flipped
        self.filtered_bbox_w = self.w_filter.update(msg.w)
        self.filtered_bbox_h = self.h_filter.update(msg.h)
        


    """ --- Methods --- """

    def _build_gps_pose_msg(self, lat: float, lon: float, alt_osd: float) -> GlobalPosition:
        """ 
        Builds geopose command message for Ardupilot.
        lat/lon - latetude and longitude of goal coordinates
        alt - altitude in meters above origin (OSD). 
        """
        
        msg = GlobalPosition()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt_osd # Altitude relative to origin location
        msg.coordinate_frame = const.FRAME_GLOBAL_REL_ALT 
        msg.header.frame_id = "map"

        msg.type_mask = (
            GlobalPosition.IGNORE_VX |
            GlobalPosition.IGNORE_VY |
            GlobalPosition.IGNORE_VZ |
            GlobalPosition.IGNORE_AFX |
            GlobalPosition.IGNORE_AFY |
            GlobalPosition.IGNORE_AFZ |
            GlobalPosition.IGNORE_YAW |
            GlobalPosition.IGNORE_YAW_RATE
                    )

        return msg

    def _build_velocity_msg(self,
        linear: tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: tuple[float, float, float] = (0.0, 0.0, 0.0)
        ) -> TwistStamped:
        """
        build velocity command message for Ardupilot.

        Linear velocities (m/s), Follows ENU convention:
            vx  = forward/backward
            vy  = right/left
            vz  = up/down 

        Angular velocities (rad/s):
            roll_rate  = rotation around X (body roll)
            pitch_rate = rotation around Y (body pitch)
            yaw_rate   = rotation around Z (body yaw)
        """
        
        msg = TwistStamped()

        vx, vy, vz = linear
        roll_rate, pitch_rate, yaw_rate = angular

        # Twist
        msg.twist.linear.x = vx   
        msg.twist.linear.y = vy  
        msg.twist.linear.z = vz

        msg.twist.angular.x = roll_rate
        msg.twist.angular.y = pitch_rate
        msg.twist.angular.z = yaw_rate

        # Header
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        return msg

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

    def _reset_vel_cmd_watchdog(self) -> None:
        self._vel_watchdog_counter = 0

    def _to_copter_flight_mode(self, internal_mode: int) -> int:
        try:
            return const.INTERNAL_TO_COPTER_MODE[internal_mode]
        except KeyError:
            raise ValueError(f"Unsupported internal mode: {internal_mode}")

    def _internal_mode_switch(self, internal_mode: int) -> None:
        """Handle side effects of switching between internal modes."""
    
        # Guided Position
        if internal_mode == const.INTERNAL_MODE_GUIDED_POS:
            self.internal_mode = const.INTERNAL_MODE_GUIDED_POS
            if self._vel_stream_loop_timer is not None:
                self._vel_stream_loop_timer.cancel()
                self._vel_stream_loop_timer = None
            return
        
        # Guided Velocity
        if internal_mode == const.INTERNAL_MODE_GUIDED_VEL:
            self.internal_mode = const.INTERNAL_MODE_GUIDED_VEL
            if self._vel_stream_loop_timer is None:
                self._vel_stream_loop_timer = self.create_timer(setting.VELOCITY_STREAM_LOOP_DT, self._velocity_stream_loop)
            return
        
        # Simple modes: Stabilize, RTL, Land
        if internal_mode in (
            const.INTERNAL_MODE_STABILIZED,
            const.INTERNAL_MODE_RTL,
            const.INTERNAL_MODE_LAND,
        ):
            self.internal_mode = internal_mode
            if self._vel_stream_loop_timer is not None:
                self._vel_stream_loop_timer.cancel()
                self._vel_stream_loop_timer = None
            return

    def arm(self) -> None:
        if self.arming_in_progress or self.current_ap_status_armed:
            return
        self._arm_request()

    def set_origin_alt(self) -> None:
        if self._origin_altitude is not None:
            return 

        self._origin_altitude = self._current_ap_geopose.position.altitude
        self.get_logger().info(f"Set origin altitude to: {self._origin_altitude}")
    
    def cmd_takeoff(self, alt_osd: float = 1.0) -> None:
        """
        Send a takeoff command to ArduPilot in GUIDED mode.

        alt_osd : float, optional
            Target takeoff altitude in meters, expressed as REL_ALT (height above
            the home/origin altitude). Default is 1.0 meter. 
        """

        # Copter must be in GUIDED mode - INTERNAL_MODE_GUIDED_POS or INTERNAL_MODE_GUIDED_VEL internal modes
        if self.current_ap_status_mode != const.COPTER_MODE_GUIDED:
            raise RuntimeError(
                f"Copter must be in GUIDED mode, but current mode is {self.current_ap_status_mode}"
            )
        if self.takeoff_in_progress:
            return

        lat = self._current_ap_geopose.position.latitude
        lon = self._current_ap_geopose.position.longitude
        alt = self._current_ap_geopose.position.altitude

        self._update_goal_pose(lat, lon, alt_osd)

        self._takeoff_request(alt_osd)

    def cmd_navigate_to(self, lat: float, lon: float, alt_osd: float) -> None:
        """
        Send a GPS navigation command to ArduPilot in GUIDED mode.
        Internal mode must be INTERNAL_MODE_GUIDED_POS.

        lat : float
            Target latitude in degrees (WGS-84).
        lon : float
            Target longitude in degrees (WGS-84).
        alt_osd : float
            Target altitude relative to the origin/home altitude (meters).
            This is NOT AGL unless you compute it yourself. ArduPilot interprets
            this value as REL_ALT (height above home).

            If using the acceleration or velocity commands, this command musch be sent once a second. Otherwise it will stop after 3s
            https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html?utm_source=chatgpt.com
        """

        # Copter must be in GUIDED mode
        if self.current_ap_status_mode != const.COPTER_MODE_GUIDED and self.internal_mode == const.INTERNAL_MODE_GUIDED_POS:
            raise RuntimeError(
                f"Expected COPTER_MODE_GUIDED for INTERNAL_MODE_GUIDED_POS, "
                f"but got copter={self.current_ap_status_mode}, internal={self.internal_mode}."
            )
        
        self._update_goal_pose(lat, lon, alt_osd)
        
        msg = self._build_gps_pose_msg(lat, lon, alt_osd)
        self._global_pos_pub.publish(msg)

    def cmd_velocity(self,
        linear: tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: tuple[float, float, float] = (0.0, 0.0, 0.0)
        ) -> None:
        """
        Send a body-frame velocity and angular rate command to ArduPilot in GUIDED mode.
        Internal mode must be INTERNAL_MODE_GUIDED_VEL

        Linear velocities (m/s), Follows ENU cenvetion:
            vx  = forward/backward
            vy  = right/left
            vz  = up/down 

        Angular velocities (rad/s):
            roll_rate  = rotation around X (body roll)
            pitch_rate = rotation around Y (body pitch)
            yaw_rate   = rotation around Z (body yaw)
        """
        # Copter must be in INTERNAL_MODE_GUIDED_VEL mode
        if self.current_ap_status_mode != const.COPTER_MODE_GUIDED or self.internal_mode != const.INTERNAL_MODE_GUIDED_VEL:
            raise RuntimeError(
                f"Expected COPTER_MODE_GUIDED ({const.COPTER_MODE_GUIDED}) for INTERNAL_MODE_GUIDED_VEL ({const.INTERNAL_MODE_GUIDED_VEL}), "
                f"but got copter={self.current_ap_status_mode}, internal={self.internal_mode}."
            )

        self._desired_linear_vel = linear
        self._desired_angular_vel = angular

        self._reset_vel_cmd_watchdog()

    def has_reached_goal(self, dist_tolerance = 1.0, alt_tolerance = 1.0) -> bool:
        # Copter must be in GUIDED or RTL mode
        if self.current_ap_status_mode not in [const.COPTER_MODE_GUIDED, const.COPTER_MODE_RTL]:
            raise RuntimeError(
                f"Copter must be in GUIDED or RTL mode, but current mode is {self.current_ap_status_mode}"
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

        # print("Flat distance: ", flat_distance, ", Altitude distance: ", alt_distance)

        if flat_distance < dist_tolerance and alt_distance < alt_tolerance:
            return True
        else:
            return False
        
    def is_takeoff_complete(self) -> bool:
        # Copter must be in GUIDED mode - INTERNAL_MODE_GUIDED_POS or INTERNAL_MODE_GUIDED_VEL internal modes
        if self.current_ap_status_mode != const.COPTER_MODE_GUIDED:
            raise RuntimeError(
                f"Copter must be in GUIDED mode, but current mode is {self.current_ap_status_mode}"
            )

        if not self.takeoff_in_progress:
            return

        is_takeoff_complete = self.has_reached_goal(alt_tolerance = 0.25)

        if is_takeoff_complete:
            self.takeoff_in_progress = False
        
        return is_takeoff_complete

    def switch_flight_mode(self, internal_mode: int) -> None:
        if self.mode_switch_in_progress:
            return

        if internal_mode not in (
            const.INTERNAL_MODE_STABILIZED,
            const.INTERNAL_MODE_GUIDED_POS,
            const.INTERNAL_MODE_GUIDED_VEL,
            const.INTERNAL_MODE_RTL,
            const.INTERNAL_MODE_LAND
        ):
            raise ValueError(f"Unsupported internal mode: {internal_mode}")

        flight_mode = self._to_copter_flight_mode(internal_mode)

        if flight_mode == self.current_ap_status_mode and internal_mode == self.internal_mode:
            return

        # Switch internal mode.
        self._internal_mode_switch(internal_mode)

        # Trigger the actual AP flight mode switch
        if flight_mode != self.current_ap_status_mode:
            self._switch_mode_request(flight_mode)
            self.mode_switch_in_progress = True

    """ --- Timer callback loop --- """

    def _velocity_stream_loop(self):

        if self._vel_watchdog_counter > setting.MAX_STALL_CYCLES_WATCHDOG:
            self.get_logger().warn("Velocity watchdog triggered! Setting all velocities to 0!")
            safe_linear  = (0.0, 0.0, 0.0)
            safe_angular = (0.0, 0.0, 1.0)
            msg = self._build_velocity_msg(safe_linear, safe_angular)
            self._vel_pub.publish(msg)
            return

        self._vel_watchdog_counter += 1

        msg = self._build_velocity_msg(self._desired_linear_vel, self._desired_angular_vel)
        self._vel_pub.publish(msg)

    def _copter_control_loop(self):

        start = time.perf_counter() # Measures relative time compared to .time

        try:
            self._controller_fsm.step()

        except Exception as e:
            self.get_logger().error(f"Copter control loop error: {e}")

        end = time.perf_counter()
        duration = end - start

        if duration > setting.COPTER_CONTROL_LOOP_DT:
            self.get_logger().warn(
                f"Control loop overrun: {duration:.4f}s (limit {setting.COPTER_CONTROL_LOOP_DT:.4f}s)"
            )


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
