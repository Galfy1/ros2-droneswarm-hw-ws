#!/usr/bin/env python3

# # based on https://github.com/ArduPilot/ardupilot/blob/2110d5324f2a8f8cd1d1fe58c4f77ee58d9530ad/Tools/ros2/ardupilot_dds_tests/ardupilot_dds_tests/copter_takeoff.py 


# # TODO vi kan vel stadig simme det her med en enkelt drone


# import rclpy
# import time

# from rclpy.node import Node
# from rclpy.time import Time
# from geographic_msgs.msg import GeoPoseStamped
# from geometry_msgs.msg import Twist

# from ardupilot_msgs.srv import ArmMotors
# from ardupilot_msgs.srv import ModeSwitch
# from ardupilot_msgs.srv import Takeoff
# from ardupilot_msgs.msg import GlobalPosition

# # Ardupilot msg/srv interfaces: https://github.com/ArduPilot/ardupilot/tree/master/Tools/ros2/ardupilot_msgs
# # Ardupilot ROS2 interfaces: https://ardupilot.org/dev/docs/ros2-interfaces.html 


# CONTROL_LOOP_DT = 0.1  # seconds
# TAKEOFF_ALT = 10.5 # feel free to change

# COPTER_MODE_GUIDED = 4 # Dont change



# def takeoff_land_loop(self):

#     alt_agl = self.current_geopose.pose.position.altitude - self.home_altitude
#     self.falg = True
#     if alt_agl < TAKEOFF_ALT and self.falg:
#         # Takeoff phase
#         self.publish_position_setpoint_global(
#             lat=self.current_geopose.pose.position.latitude,
#             lon=self.current_geopose.pose.position.longitude,
#             alt=TAKEOFF_ALT + self.home_altitude,
#             speed=1.0,
#             yaw=0.0
#         )
#         self.falg = False
#     else:
#         # Land phase
#         return
#         # TODO self.land()


# class CopterTakeoff(Node):

#     def __init__(self) -> None:
#         """Initialise the node."""
#         super().__init__("copter_takeoff_and_land")

#         # Configure QoS profile for publishing and subscribing
#         qos_profile = rclpy.qos.QoSProfile(
#             reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
#             durability=rclpy.qos.DurabilityPolicy.VOLATILE,
#             depth=1
#         )

#         # Create service clients
#         self.arm_client = self.create_client(ArmMotors, "/ap/arm_motors")
#         while not self.arm_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('arm service not available, waiting again...')

#         self.mode_switch_client = self.create_client(ModeSwitch, "/ap/mode_switch")
#         while not self.mode_switch_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('takeoff service not available, waiting again...')

#         self.takeoff_client = self.create_client(Takeoff, "/ap/experimental/takeoff")
#         while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('takeoff service not available, waiting again...')

#         # Create subscribers
#         self.geopose_subscriber = self.create_subscription(GeoPoseStamped, "/ap/geopose/filtered", self.geopose_callback, qos_profile)

#         # Create publishers
#         self.cmd_gps_pose_publisher = self.create_publisher(GlobalPosition, "/ap/cmd_gps_pose", qos_profile)
    

#         # Create a timer to publish control commands
#         self.timer = self.create_timer(CONTROL_LOOP_DT, self.control_loop_callback)

#         # Initialize variables
#         self.current_geopose = GeoPoseStamped()
#         self.switched_to_guided = False
#         self.arming_complete = False
#         self.disarming_complete = False
#         self.ardupilot_takeoff_complete = False
#         self.pre_flight_wait_complete = False

#         self.start_time = None
#         self.home_altitude = None


#     ##################### METHODS #####################

#     def geopose_callback(self, msg: GeoPoseStamped): # https://docs.ros.org/en/noetic/api/geographic_msgs/html/msg/GeoPoseStamped.html
#         """Process a GeoPose message."""
#         stamp = msg.header.stamp
#         if stamp.sec: # only process if we have a valid timestamp
#             # self.get_logger().info("From AP : Geopose [sec:{}, nsec: {}]".format(stamp.sec, stamp.nanosec))
#             # Store current geopose
#             self.current_geopose = msg
            
#     # def geopose_callback(self, msg: GeoPoseStamped):
#     #     """Process a GeoPose message."""
#     #     # Convert message timestamp to rclpy Time
#     #     msg_time = Time.from_msg(msg.header.stamp)

#     #     # Only process if timestamp is newer than our stored start_time
#     #     if self.start_time is not None and msg_time > self.start_time:
#     #         print("Received a newer GeoPose message.")
#     #         self.current_geopose = msg
#     #     else:
#     #         print("Skipping old GeoPose message.")



#     def arm(self):
#         self.arming_complete = False
#         req = ArmMotors.Request()
#         req.arm = True
#         future = self.arm_client.call_async(req)
#         future.add_done_callback(self.arm_client_callback)

#     def arm_client_callback(self, future):
#         result = future.result()
#         # Set flag True if succesfully armed:
#         self.arming_complete = result.result # (yes, there is a result field in ArmMotors.srv)

#     def disarm(self):
#         self.disarming_complete = False
#         req = ArmMotors.Request()
#         req.arm = False # false for "disarm"
#         future = self.arm_client.call_async(req)
#         future.add_done_callback(self.disarm_client_callback)

#     def disarm_client_callback(self, future):
#         result = future.result()
#         # Set flag True if succesfully disarmed:
#         self.disarming_complete = result.result # (yes, there is a result field in ArmMotors.srv)

#     def switch_to_guided_mode(self):
#         self.switched_to_guided = False # reset flag
#         req = ModeSwitch.Request()
#         req.mode = COPTER_MODE_GUIDED
#         future = self.mode_switch_client.call_async(req)
#         future.add_done_callback(self.switch_to_guided_mode_client_callback)

#     def switch_to_guided_mode_client_callback(self, future):
#         result = future.result() # TODO de havde future.result().result i deres kode.. men det virker forkert?? kom tilbage hvis det ikke virker

#         print(result.curr_mode)
#         # Set flag True if succesfully switched to guided mode:
#         self.switched_to_guided = (result.status) or (result.curr_mode == COPTER_MODE_GUIDED)

#     # def ardupilot_takeoff(self, alt):
#     #     self.ardupilot_takeoff_complete = False
#     #     req = Takeoff.Request()
#     #     req.alt = alt
#     #     future = self.takeoff_client.call_async(req)
#     #     future = self.add_done_callback(self.ardupilot_takeoff_client_callback)

#     def ardupilot_takeoff(self, alt: float):
#         self.ardupilot_takeoff_complete = False
#         req = Takeoff.Request()
#         req.alt = float(alt)
#         future = self.takeoff_client.call_async(req)
#         future.add_done_callback(self.ardupilot_takeoff_client_callback)

#     def land(self):
#         pass
#         # TODO skal gøres ved at sætte den i "land" mode.. aka switch_to_guided_mode skal nok laves om til at kunne tage en mode parameter

#     def ardupilot_takeoff_client_callback(self, future):
#         result = future.result()
#         # Set flag True if succesfully took off:
#         self.ardupilot_takeoff_complete = result.status

#     def alt_amsl2agl(self):
#         # Convert the altitude in the msg.geopose from AMSL to AGL
#         pass


 
#     def publish_position_setpoint_global(self, lat: float, lon: float, alt: float, speed: float = 1.0, yaw: float = 0.0):
#         # TODO THE ALT SIGN IS REVERSED COMPARED TO PX4

#         msg = GlobalPosition()
#         msg.latitude = lat
#         msg.longitude = lon
#         msg.altitude = alt
#         msg.yaw = yaw
#         velocity = Twist()
#         velocity.linear.x = speed
#         velocity.linear.y = speed
#         velocity.linear.z = speed
#         msg.velocity = velocity
#         self.cmd_gps_pose_publisher.publish(msg)
    
    
#     def pre_takeoff_wait(self):
        
#         # Wait for 5 seconds before starting takeoff procedure
#         wait_duration = 4.0  # seconds
#         current_time = self.get_clock().now()
#         elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
#         # print(elapsed_time)
#         if elapsed_time >= wait_duration:
#             self.pre_flight_wait_complete = True
#             # self.get_logger().info("Pre-flight wait complete. Proceeding with takeoff.") 
#         # else:
#         #     self.get_logger().info(f"Waiting for pre-flight wait to complete: {elapsed_time:.2f}/{wait_duration} seconds")
        

#     ##################### MAIN CONTROL LOOP #####################

#     def control_loop_callback(self) -> None:

#         cl_start_time = self.get_clock().now()

#         if self.start_time is None:
#             self.start_time = self.get_clock().now()

#         if not self.pre_flight_wait_complete:
#             self.pre_takeoff_wait()
#             return 

#         # Fetch and store current home altitude
#         if self.home_altitude is None:        
#             self.home_altitude = self.current_geopose.pose.position.altitude
#             self.get_logger().info(f"Current home altitude: {self.home_altitude} m")

#         # STEP 1: Switch to guided mode if not already in guided mode
#         if not self.switched_to_guided:
#             #self.get_logger().info("Switching to GUIDED mode...")
#             self.switch_to_guided_mode()
#             return  # wait for next loop iteration
#         #self.get_logger().info("Copter is in GUIDED mode.")

#         # STEP 2: Arm motors if not already armed
#         if not self.arming_complete:
#             #self.get_logger().info("Arming motors...")
#             self.arm()
#             return  # wait for next loop iteration
        
#         # Step 3: Do initial "ardupilot takeoff"
#         # (Not sure if ardupilot require using its "takeoff" service, before we can publish pos setpoints - or we if can publish pos directly after arming like on PX4
#         # Just to make sure, we run an initial "ardupilot takeoff" command, to get in the air.)
#         if not self.ardupilot_takeoff_complete:
#             self.switch_to_guided = False # reset flag
#             self.ardupilot_takeoff(1.0) # 1 meter
#             return

#         if not self.switched_to_guided:
#             #self.get_logger().info("Switching to GUIDED mode...")
#             self.switch_to_guided_mode()
#             return  # wait for next loop iteration
        
    
#         # STEP 4: do whatever you want in your "application":
#         takeoff_land_loop(self)

#         # ERROR CHECKING - Make sure control loop processing time does not exceed CONTROL_LOOP_DT:
#         cl_end_time = self.get_clock().now()
#         elapsed_time_s = (cl_end_time - cl_start_time).nanoseconds / 1e9
#         if elapsed_time_s > CONTROL_LOOP_DT:
#             self.get_logger().warn(f"Control loop overrun: {elapsed_time_s:.4f} seconds")










# def main(args=None) -> None:
#     print('Starting copter takeoff example node...')
#     rclpy.init(args=args)

#     copter_takeoff = CopterTakeoff()
#     rclpy.spin(copter_takeoff)

#     copter_takeoff.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     try:
#         main()
#     except Exception as e:
#         print(e)


# """
# Run takeoff test on Copter, then move to a GPS waypoint.

# Warning - This is NOT production code; it's a simple demo of capability.
# """

# import rclpy
# import time

# from rclpy.node import Node
# from geographic_msgs.msg import GeoPoseStamped
# from ardupilot_msgs.srv import ArmMotors
# from ardupilot_msgs.srv import ModeSwitch
# from ardupilot_msgs.srv import Takeoff
# from ardupilot_msgs.msg import GlobalPosition
# from geometry_msgs.msg import Twist

# COPTER_MODE_GUIDED = 4
# TAKEOFF_ALT = 10.5
# TOLERANCE = 0.5


# class CopterTakeoff(Node):
#     """Copter takeoff using guided control."""

#     def __init__(self):
#         super().__init__("copter_takeoff")

#         self.declare_parameter("arm_topic", "/ap/arm_motors")
#         self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
#         self._client_arm = self.create_client(ArmMotors, self._arm_topic)
#         while not self._client_arm.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('arm service not available, waiting again...')

#         self.declare_parameter("mode_topic", "/ap/mode_switch")
#         self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
#         self._client_mode_switch = self.create_client(ModeSwitch, self._mode_topic)
#         while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('mode switch service not available, waiting again...')

#         self.declare_parameter("takeoff_service", "/ap/experimental/takeoff")
#         self._takeoff_topic = self.get_parameter("takeoff_service").get_parameter_value().string_value
#         self._client_takeoff = self.create_client(Takeoff, self._takeoff_topic)
#         while not self._client_takeoff.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('takeoff service not available, waiting again...')

#         self.declare_parameter("geopose_topic", "/ap/geopose/filtered")
#         self._geopose_topic = self.get_parameter("geopose_topic").get_parameter_value().string_value
#         qos = rclpy.qos.QoSProfile(
#             reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
#             durability=rclpy.qos.DurabilityPolicy.VOLATILE,
#             depth=1,
#         )

#         self._subscription_geopose = self.create_subscription(
#             GeoPoseStamped, self._geopose_topic, self.geopose_cb, qos
#         )

#         self._cur_geopose = GeoPoseStamped()

#         # ---- Added publisher for GUIDED GPS position target ----
#         self._cmd_gps_pose_pub = self.create_publisher(GlobalPosition, "/ap/cmd_gps_pose",
#             qos,
#         )

#     def geopose_cb(self, msg: GeoPoseStamped):
#         stamp = msg.header.stamp
#         if stamp.sec:
#             # self.get_logger().info(f"From AP : Geopose [sec:{stamp.sec}, nsec:{stamp.nanosec}]")
#             self._cur_geopose = msg

#     def arm(self):
#         req = ArmMotors.Request()
#         req.arm = True
#         future = self._client_arm.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()

#     def arm_with_timeout(self, timeout: rclpy.duration.Duration):
#         armed = False
#         start = self.get_clock().now()
#         while not armed and self.get_clock().now() - start < timeout:
#             armed = self.arm().result
#             time.sleep(1)
#         return armed

#     def switch_mode(self, mode):
#         req = ModeSwitch.Request()
#         assert mode in [COPTER_MODE_GUIDED]
#         req.mode = mode
#         future = self._client_mode_switch.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()

#     def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
#         is_in_desired_mode = False
#         start = self.get_clock().now()
#         while not is_in_desired_mode and self.get_clock().now() - start < timeout:
#             result = self.switch_mode(desired_mode)
#             is_in_desired_mode = result.status or result.curr_mode == desired_mode
#             time.sleep(1)
#         print(is_in_desired_mode)
#         return is_in_desired_mode

#     def takeoff(self, alt):
#         req = Takeoff.Request()
#         req.alt = alt
#         future = self._client_takeoff.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()

#     def takeoff_with_timeout(self, alt: int, timeout: rclpy.duration.Duration):
#         takeoff_success = False
#         start = self.get_clock().now()
#         while not takeoff_success and self.get_clock().now() - start < timeout:
#             result = self.takeoff(alt)
#             takeoff_success = result.status
#             time.sleep(1)
#         return takeoff_success

#     def get_cur_geopose(self):
#         return self._cur_geopose

#     # ---- NEW: publishing GPS target ----
#     def publish_global_position_target(self, lat, lon, alt_amsl, speed=2.0):
#         msg = GlobalPosition()
#         msg.latitude = float(lat)
#         msg.longitude = float(lon)
#         msg.altitude = float(alt_amsl)

#         msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_TERRAIN_ALT

#         # --- Type mask (ignore everything except lat/lon/alt) ---
#         msg.type_mask = (
#             GlobalPosition.IGNORE_VX |
#             GlobalPosition.IGNORE_VY |
#             GlobalPosition.IGNORE_VZ |
#             GlobalPosition.IGNORE_AFX |
#             GlobalPosition.IGNORE_AFY |
#             GlobalPosition.IGNORE_AFZ |
#             GlobalPosition.IGNORE_YAW |
#             GlobalPosition.IGNORE_YAW_RATE
#         )

#         self._cmd_gps_pose_pub.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = CopterTakeoff()

#     try:
#         if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
#             raise RuntimeError("Unable to switch to guided mode")

#         if not node.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
#             raise RuntimeError("Unable to arm")

#         if not node.takeoff_with_timeout(TAKEOFF_ALT, rclpy.duration.Duration(seconds=20)):
#             raise RuntimeError("Unable to takeoff")

#         home_alt = node.get_cur_geopose().pose.position.altitude
#         node.get_logger().info(f"Taking off... Home alt: {home_alt:.2f} m")

#         is_ascending = True
#         while is_ascending:
#             node.get_logger().info(f"Ascending..., current alt: {(node.get_cur_geopose().pose.position.altitude - home_alt):.2f} m")
#             rclpy.spin_once(node)
#             time.sleep(1)
#             is_ascending = (node.get_cur_geopose().pose.position.altitude - home_alt) < (TAKEOFF_ALT - TOLERANCE)

#         node.get_logger().info("Reached takeoff altitude. Moving to GPS waypoint...")

#         if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
#             raise RuntimeError("Unable to switch to guided mode")

#         # ---- TARGET WAYPOINT (about 5m NE) ----
#         cur = node.get_cur_geopose().pose.position
#         TARGET_LAT = cur.latitude + 0.00005
#         TARGET_LON = cur.longitude + 0.00005
#         TARGET_ALT = cur.altitude

#         while rclpy.ok():
#             rclpy.spin_once(node)
#             cur = node.get_cur_geopose().pose.position

#             node.publish_global_position_target(TARGET_LAT, TARGET_LON, TARGET_ALT)

#             dlat = (cur.latitude - TARGET_LAT) * 111320.0
#             dlon = (cur.longitude - TARGET_LON) * 111320.0
#             dist = (dlat**2 + dlon**2) ** 0.5

#             node.get_logger().info(f"Moving to waypoint. Distance left: {dist:.2f} m")

#             if dist < 1.5:
#                 node.get_logger().info("Arrived at GPS waypoint!")
#                 break

#             time.sleep(0.1)

#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()





### THIS CODE WORKS ###


# import rclpy
# import time
# from rclpy.node import Node

# from ardupilot_msgs.srv import ArmMotors
# from ardupilot_msgs.srv import ModeSwitch
# from ardupilot_msgs.srv import Takeoff

# from geographic_msgs.msg import GeoPoseStamped
# from geometry_msgs.msg import TwistStamped

# COPTER_MODE_GUIDED = 4
# TAKEOFF_ALT = 10.0
# TOLERANCE = 0.5


# class CopterTakeoff(Node):

#     def __init__(self):
#         super().__init__("copter_takeoff")

#         # --- Services ---
#         self.arm_client = self.create_client(ArmMotors, "/ap/arm_motors")
#         self.mode_client = self.create_client(ModeSwitch, "/ap/mode_switch")
#         self.takeoff_client = self.create_client(Takeoff, "/ap/experimental/takeoff")

#         for client, name in [
#             (self.arm_client, "Arm"),
#             (self.mode_client, "Mode switch"),
#             (self.takeoff_client, "Takeoff"),
#         ]:
#             while not client.wait_for_service(timeout_sec=1.0):
#                 self.get_logger().info(f"{name} service not available, waiting...")

#         # --- Pose feedback ---
#         qos = rclpy.qos.QoSProfile(
#             depth=1,
#             reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
#             durability=rclpy.qos.DurabilityPolicy.VOLATILE,
#         )

#         self.pose_sub = self.create_subscription(
#             GeoPoseStamped,
#             "/ap/geopose/filtered",
#             self.pose_cb,
#             qos,
#         )
#         self.current_pose = GeoPoseStamped()

#         # --- Velocity publisher (correct!) ---
#         self.vel_pub = self.create_publisher(
#             TwistStamped,
#             "/ap/cmd_vel",
#             10
#         )

#     def pose_cb(self, msg):
#         self.current_pose = msg

#     # ----- Service wrappers -----

#     def arm(self):
#         req = ArmMotors.Request()
#         req.arm = True
#         future = self.arm_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result().result

#     def set_mode(self, mode):
#         req = ModeSwitch.Request()
#         req.mode = mode
#         future = self.mode_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result().status

#     def takeoff(self, alt):
#         req = Takeoff.Request()
#         req.alt = alt
#         future = self.takeoff_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result().status

#     # ----- MAIN LOGIC -----

#     def run(self):

#         # 1. GUIDED mode
#         self.get_logger().info("Switching to GUIDED mode...")
#         if not self.set_mode(COPTER_MODE_GUIDED):
#             raise RuntimeError("Failed to switch mode")

#         # 2. Arm
#         self.get_logger().info("Arming motors...")
#         if not self.arm():
#             raise RuntimeError("Failed to arm")

#         # 3. Takeoff
#         self.get_logger().info("Taking off...")
#         if not self.takeoff(TAKEOFF_ALT):
#             raise RuntimeError("Failed to takeoff")

#         time.sleep(2)

#         # 4. Wait for altitude
#         home_alt = self.current_pose.pose.position.altitude
#         while True:
#             alt = self.current_pose.pose.position.altitude - home_alt
#             self.get_logger().info(f"Current climb: {alt:.1f} m")
#             if alt >= TAKEOFF_ALT - TOLERANCE:
#                 break
#             rclpy.spin_once(self)
#             time.sleep(0.2)

#         self.get_logger().info("Reached takeoff altitude!")

#         # ----- MOVEMENT USING TwistStamped -----

#         self.get_logger().info("Moving drone...")

#         move = TwistStamped()
#         move.header.frame_id = "base_link"

#         move.twist.linear.x = 2.0   # forward
#         move.twist.linear.y = 1.0   # right
#         move.twist.linear.z = 0.0

#         duration = 5  # seconds
#         t0 = time.time()

#         while time.time() - t0 < duration:
#             move.header.stamp = self.get_clock().now().to_msg()
#             self.vel_pub.publish(move)
#             rclpy.spin_once(self)
#             time.sleep(0.1)

#         # stop motion
#         stop = TwistStamped()
#         stop.header.stamp = self.get_clock().now().to_msg()
#         self.vel_pub.publish(stop)

#         self.get_logger().info("Movement complete.")


# def main(args=None):
#     rclpy.init(args=args)
#     node = CopterTakeoff()
#     try:
#         node.run()
#     except Exception as e:
#         node.get_logger().error(str(e))
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()







import rclpy
import time
from rclpy.node import Node

from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from ardupilot_msgs.msg import GlobalPosition
from geographic_msgs.msg import GeoPoseStamped

COPTER_MODE_GUIDED = 4
TAKEOFF_ALT = 10.0
TOLERANCE = 0.5


GRAYHOUND_TRACK = point.Point(latitude=-35.345996, longitude=149.159017, altitude=0.575)
CMAC = point.Point(latitude=-35.3627010, longitude=149.1651513, altitude=0.585)

class CopterTakeoff(Node):

    def __init__(self):
        super().__init__("copter_takeoff")

        # --- Services ---
        self.arm_client = self.create_client(ArmMotors, "/ap/arm_motors")
        self.mode_client = self.create_client(ModeSwitch, "/ap/mode_switch")
        self.takeoff_client = self.create_client(Takeoff, "/ap/experimental/takeoff")

        for client, name in [
            (self.arm_client, "Arm"),
            (self.mode_client, "Mode switch"),
            (self.takeoff_client, "Takeoff")
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"{name} service not available, waiting...")

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )

        # --- Pose feedback ---
        self.pose_sub = self.create_subscription(
            GeoPoseStamped,
            "/ap/geopose/filtered",
            self.pose_cb,
            qos
        )
        self.current_pose = GeoPoseStamped()

        # --- GPS setpoint publisher ---
        self.gps_pub = self.create_publisher(
            GlobalPosition,
            "/ap/cmd_gps_pose",
            10
        )

    def pose_cb(self, msg):
        self.current_pose = msg

    # ----- Service wrappers -----

    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().result

    def set_mode(self, mode):
        req = ModeSwitch.Request()
        req.mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().status

    def takeoff(self, alt):
        req = Takeoff.Request()
        req.alt = alt
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().status

    # ----- GPS SETPOINT -----

    def send_goal_position(self, goal_global_pos):
        """Send goal position. Must be in guided for this to work."""
        self._global_pos_pub.publish(goal_global_pos)

    # ----- MAIN LOGIC -----

    def run(self):

        # 1. GUIDED MODE
        self.get_logger().info("Switching to GUIDED mode...")
        if not self.set_mode(COPTER_MODE_GUIDED):
            raise RuntimeError("Failed to switch mode")

        # 2. ARM
        self.get_logger().info("Arming motors...")
        if not self.arm():
            raise RuntimeError("Failed to arm")

        # 3. TAKEOFF
        self.get_logger().info("Taking off...")
        if not self.takeoff(TAKEOFF_ALT):
            raise RuntimeError("Failed to takeoff")

        time.sleep(2)

        # Wait until reaching altitude
        home_alt = self.current_pose.pose.position.altitude
        print("Taking off... Home alt: {:.2f} m".format(home_alt))
        while True:
            rclpy.spin_once(self)
            alt = self.current_pose.pose.position.altitude - home_alt
            self.get_logger().info(f"Current climb: {alt:.1f} m")
            if alt >= TAKEOFF_ALT - TOLERANCE:
                break
            time.sleep(0.2)

        self.get_logger().info("Reached takeoff altitude!")


        if not node.switch_mode_with_timeout(PLANE_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to guided mode")


        # ----- PUBLISH GPS SETPOINT UNTIL REACHED -----
        while rclpy.ok():

             # Send a guided WP with location, frame ID, alt frame
            goal_pos = GlobalPosition()
            goal_pos.latitude = GRAYHOUND_TRACK.latitude
            goal_pos.longitude = GRAYHOUND_TRACK.longitude
            DESIRED_AGL = 60
            goal_pos.altitude = GRAYHOUND_TRACK.altitude * 1000 + DESIRED_AGL
            goal_pos.coordinate_frame = FRAME_GLOBAL_INT
            goal_pos.header.frame_id = "map"


            self.send_goal_position(goal_pos)

            rclpy.spin_once(self)

            # compute distance (rough GPS → meters conversion)
            cur = self.current_pose.pose.position
            dlat = (cur.latitude - TARGET_LAT) * 111320.0
            dlon = (cur.longitude - TARGET_LON) * 111320.0
            dist = (dlat**2 + dlon**2) ** 0.5

            self.get_logger().info(f"Distance to waypoint: {dist:.2f} m")

            if dist < 1.5:
                self.get_logger().info("Arrived at GPS waypoint!")
                break

            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = CopterTakeoff()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Joy
# from pynput import keyboard


# class KeyboardJoyPublisher(Node):
#     def __init__(self):
#         super().__init__("keyboard_joy_publisher")
#         self.publisher_ = self.create_publisher(Joy, "/ap/joy", 10)

#         self.axes = [0.0, 0.0, 0.0, 0.0]

#         # Create a timer to publish the message every 1 second
#         self.timer = self.create_timer(1, self.publish_joy)

#         # Register the keyboard listener
#         self.listener = keyboard.Listener(
#             on_press=self.on_press, on_release=self.on_release
#         )
#         self.listener.start()

#     def on_press(self, key):
#         if key == "w":
#             self.axes[3] = 1.0
#         elif key == "a":
#             self.axes[1] = -1.0
#         elif key == "s":
#             self.axes[2] = 1.0
#         elif key == "d":
#             self.axes[0] = 1.0

#     def on_release(self, key):
#         if key == "w":
#             self.axes[3] = 0.0
#         elif key == "a":
#             self.axes[1] = 0.0
#         elif key == "s":
#             self.axes[2] = 0.0
#         elif key == "d":
#             self.axes[0] = 0.0

#     def publish_joy(self):
#         joy_msg = Joy()
#         joy_msg.axes = self.axes

#         now = self.get_clock().now()
#         joy_msg.header.stamp = now.to_msg()
#         joy_msg.header.stamp.nanosec = now.nanoseconds % 1000000000

#         self.publisher_.publish(joy_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     keyboard_joy_publisher = KeyboardJoyPublisher()
#     rclpy.spin(keyboard_joy_publisher)
#     keyboard_joy_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()






# import math
# import rclpy
# import time

# from rclpy.node import Node
# from ardupilot_msgs.msg import GlobalPosition
# from geographic_msgs.msg import GeoPoseStamped
# from geopy import distance
# from geopy import point
# from ardupilot_msgs.srv import ArmMotors
# from ardupilot_msgs.srv import ModeSwitch
# from geographic_msgs.msg import GeoPointStamped


# COPTER_MODE_GUIDED = 4

# FRAME_GLOBAL_INT = 5

# # Hard code some known locations
# # Note - Altitude in geopy is in km!
# GRAYHOUND_TRACK = point.Point(latitude=-35.345996, longitude=149.159017, altitude=0.575)
# CMAC = point.Point(latitude=-35.3627010, longitude=149.1651513, altitude=0.585)


# class CopterTakeoff(Node):
#     """Plane follow waypoints using guided control."""

#     def __init__(self):
#         """Initialise the node."""
#         super().__init__("plane_waypoint_follower")

#         self.declare_parameter("arm_topic", "/ap/arm_motors")
#         self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
#         self._client_arm = self.create_client(ArmMotors, self._arm_topic)
#         while not self._client_arm.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('arm service not available, waiting again...')

#         self.declare_parameter("mode_topic", "/ap/mode_switch")
#         self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
#         self._client_mode_switch = self.create_client(ModeSwitch, self._mode_topic)
#         while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('mode switch service not available, waiting again...')

#         self.declare_parameter("global_position_topic", "/ap/cmd_gps_pose")
#         self._global_pos_topic = self.get_parameter("global_position_topic").get_parameter_value().string_value
#         self._global_pos_pub = self.create_publisher(GlobalPosition, self._global_pos_topic, 1)

#         # Create subscriptions after services are up
#         self.declare_parameter("geopose_topic", "/ap/geopose/filtered")
#         self._geopose_topic = self.get_parameter("geopose_topic").get_parameter_value().string_value
#         qos = rclpy.qos.QoSProfile(
#             reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=1
#         )

#         self._subscription_geopose = self.create_subscription(GeoPoseStamped, self._geopose_topic, self.geopose_cb, qos)
#         self._cur_geopose = GeoPoseStamped()

#         self.declare_parameter("goal_topic", "/ap/goal_lla")
#         self._goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
#         qos = rclpy.qos.QoSProfile(
#             reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
#             durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
#             depth=1,
#         )

#         self._subscription_goal = self.create_subscription(GeoPointStamped, self._goal_topic, self.goal_cb, qos)
#         self._cur_goal = GeoPointStamped()

#     def geopose_cb(self, msg: GeoPoseStamped):
#         """Process a GeoPose message."""
#         stamp = msg.header.stamp
#         if stamp.sec:
#             self.get_logger().info("From AP : Geopose [sec:{}, nsec: {}]".format(stamp.sec, stamp.nanosec))

#             # Store current state
#             self._cur_geopose = msg

#     def goal_cb(self, msg: GeoPointStamped):
#         """Process a Goal message."""
#         stamp = msg.header.stamp
#         self.get_logger().info(
#             "From AP : Goal [sec:{}, nsec: {}, lat:{} lon:{}]".format(
#                 stamp.sec, stamp.nanosec, msg.position.latitude, msg.position.longitude
#             )
#         )

#         # Store current state
#         self._cur_goal = msg

#     def arm(self):
#         req = ArmMotors.Request()
#         req.arm = True
#         future = self._client_arm.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result()

#     def arm_with_timeout(self, timeout: rclpy.duration.Duration):
#         """Try to arm. Returns true on success, or false if arming fails or times out."""
#         armed = False
#         start = self.get_clock().now()
#         while not armed and self.get_clock().now() - start < timeout:
#             armed = self.arm().result
#             time.sleep(1)
#         return armed

#     def switch_mode(self, mode):
#         req = ModeSwitch.Request()
#         req.mode = mode
#         future = self.mode_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result().status

#     def takeoff(self, alt):
#         req = Takeoff.Request()
#         req.alt = alt
#         future = self.takeoff_client.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         return future.result().status


#     def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
#         """Try to switch mode. Returns true on success, or false if mode switch fails or times out."""
#         is_in_desired_mode = False
#         start = self.get_clock().now()
#         while not is_in_desired_mode:
#             result = self.switch_mode(desired_mode)
#             # Handle successful switch or the case that the vehicle is already in expected mode
#             is_in_desired_mode = result.status or result.curr_mode == desired_mode
#             time.sleep(1)

#         return is_in_desired_mode

#     def get_cur_geopose(self):
#         """Return latest geopose."""
#         return self._cur_geopose

#     def get_cur_goal(self):
#         """Return latest goal."""
#         return self._cur_goal

#     def send_goal_position(self, goal_global_pos):
#         """Send goal position. Must be in guided for this to work."""
#         self._global_pos_pub.publish(goal_global_pos)


# def achieved_goal(goal_global_pos, cur_geopose):
#     """Return true if the current position (LLH) is close enough to the goal (within the orbit radius)."""
#     # Use 3D geopy distance calculation
#     # https://geopy.readthedocs.io/en/stable/#module-geopy.distance
#     goal_lat = goal_global_pos

#     p1 = (goal_global_pos.latitude, goal_global_pos.longitude, goal_global_pos.altitude)
#     cur_pos = cur_geopose.pose.position
#     p2 = (cur_pos.latitude, cur_pos.longitude, cur_pos.altitude)

#     flat_distance = distance.distance(p1[:2], p2[:2]).m
#     euclidian_distance = math.sqrt(flat_distance**2 + (p2[2] - p1[2]) ** 2)
#     print(f"Goal is {euclidian_distance} meters away")
#     return euclidian_distance < 150


# def going_to_goal(goal_global_pos, cur_goal):
#     p1 = (goal_global_pos.latitude, goal_global_pos.longitude, goal_global_pos.altitude)
#     cur_pos_lla = cur_goal.position
#     p2 = (cur_pos_lla.latitude, cur_pos_lla.longitude, cur_pos_lla.altitude)

#     flat_distance = distance.distance(p1[:2], p2[:2]).m
#     euclidian_distance = math.sqrt(flat_distance**2 + (p2[2] - p1[2]) ** 2)
#     print(f"Commanded and received goal are {euclidian_distance} meters away")
#     return euclidian_distance < 1


# def main(args=None):
#     """Node entry point."""
#     rclpy.init(args=args)
#     node = CopterTakeoff()
#     try:
#         # Block till armed, which will wait for EKF3 to initialize
#         if not node.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
#             raise RuntimeError("Unable to arm")

#         # Block till in takeoff
#         if not node.switch_mode_with_timeout(COPTER_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
#             raise RuntimeError("Unable to switch to takeoff mode")

#         node.takeoff

#             # Hard code waiting in takeoff to reach operating altitude of 630m
#             # This is just a hack because geopose is reported with absolute rather than relative altitude,
#             # and this node doesn't have access to the terrain data
#             is_ascending_to_takeoff_alt = node.get_cur_geopose().pose.position.altitude < CMAC.altitude * 1000 + 45

#         if is_ascending_to_takeoff_alt:
#             raise RuntimeError("Failed to reach takeoff altitude")

#         if not node.switch_mode_with_timeout(PLANE_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
#             raise RuntimeError("Unable to switch to guided mode")

#         # Send a guided WP with location, frame ID, alt frame
#         goal_pos = GlobalPosition()
#         goal_pos.latitude = GRAYHOUND_TRACK.latitude
#         goal_pos.longitude = GRAYHOUND_TRACK.longitude
#         DESIRED_AGL = 60
#         goal_pos.altitude = GRAYHOUND_TRACK.altitude * 1000 + DESIRED_AGL
#         goal_pos.coordinate_frame = FRAME_GLOBAL_INT
#         goal_pos.header.frame_id = "map"

#         node.send_goal_position(goal_pos)

#         start = node.get_clock().now()
#         has_achieved_goal = False
#         is_going_to_goal = False
#         while not has_achieved_goal and node.get_clock().now() - start < rclpy.duration.Duration(seconds=120):
#             rclpy.spin_once(node)
#             is_going_to_goal = going_to_goal(goal_pos, node.get_cur_goal())
#             has_achieved_goal = achieved_goal(goal_pos, node.get_cur_geopose())
#             time.sleep(1.0)

#         if not is_going_to_goal:
#             raise RuntimeError("Unable to go to goal location")
#         if not has_achieved_goal:
#             raise RuntimeError("Unable to achieve goal location")

#         print("Goal achieved")

#     except KeyboardInterrupt:
#         pass

#     # Destroy the node explicitly.
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()