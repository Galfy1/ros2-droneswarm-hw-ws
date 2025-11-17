#!/usr/bin/env python3

import rclpy
import time

from rclpy.node import Node
from std_srvs.srv import Trigger # https://docs.ros.org/en/melodic/api/std_srvs/html/srv/Trigger.html
from ardupilot_msgs.srv import ArmMotors # https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/ardupilot_msgs/srv/ArmMotors.srv
from ardupilot_msgs.srv import ModeSwitch # https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/ardupilot_msgs/srv/ModeSwitch.srv
from ardupilot_msgs.srv import Takeoff # https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/ardupilot_msgs/srv/Takeoff.srv
from ardupilot_msgs.msg import Status # https://github.com/ArduPilot/ardupilot/blob/master/Tools/ros2/ardupilot_msgs/msg/Status.msg
# from sensor_msgs.msg import NavSatFix 



# https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE
COPTER_MODE_STABILIZE = 0
COPTER_MODE_GUIDED = 4

TAKEOFF_ALT = 1.0


class CopterControllerNode(Node):

    def __init__(self):
        """ Initialize the copter controller node """
        super().__init__("copter_controller")

    
        # --- Create callback timers --- 
        self.copter_controll_loop_timer = self.create_timer(0.1, self.copter_control_loop)  # 10 Hz
        self.prearm_check_timer = self.create_timer(2.0, self.prearm_check)  # check every 2s

        # --- Services ---
        self._client_prearm = self.create_client(Trigger, "/ap/prearm_check")
        self._client_arm = self.create_client(ArmMotors, "/ap/arm_motors")
        self.mode_switch_client = self.create_client(ModeSwitch, "/ap/mode_switch")
        self.takeoff_client = self.create_client(Takeoff, "/ap/experimental/takeoff")

        while not self._client_prearm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('prearm service not available, waiting again...')

        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        while not self.mode_switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')

        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')


        # Configure QoS profile for publishing and subscribing
        qos_profile = rclpy.qos.QoSProfile(
            reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
            durability = rclpy.qos.DurabilityPolicy.VOLATILE,
            depth = 1
        )

        # --- Topics ---
        # Subscribers
        # self.NavSatFix_sub = self.create_subscription(NavSatFix, "/ap/navsat", self._navsatfix_callback, qos_profile)
        self.ap_status_sub = self.create_subscription(Status, "/ap/status", self._ap_status_callback, qos_profile)

       


        # Initializing copter controller flags
        self.prearm_check_in_progress = False
        self.is_prearm_ok = False
        self.arming_in_progress = False
        self.mode_switch_in_progress = False
        self.takeoff_in_progress = False
        self.is_takeoff_ok = False


        # Updated by _ap_status_callback() from /ap/status topic
        self.current_ap_status_mode = COPTER_MODE_STABILIZE # Copter starts in stabilized mode.
        self.current_ap_status_armed = False
        self.current_ap_status_flying = False
        # self.curr_navsatfix_status = None 

        self._pending_mode = None


    """ --- Service Methods  --- """

    def prearm_check(self):
        if self.prearm_check_in_progress:
            return

        self.prearm_check_in_progress = True

        req = Trigger.Request()
        future = self._client_prearm.call_async(req)
        future.add_done_callback(self._on_prearm_check_complete)

    def _on_prearm_check_complete(self, future):

        # If we already passed prearm, ignore late results
        if self.prearm_check_timer is None or self.is_prearm_ok:
            return
        
        try:
            self.is_prearm_ok = future.result().success
        except Exception:
            self.is_prearm_ok = False

        self.prearm_check_in_progress = False

        # Stop checking if success
        if self.is_prearm_ok:
            self.get_logger().info("Prearm OK. Stopping prearm checks.")
            self.prearm_check_timer.cancel()
            self.prearm_check_timer = None
            return

        self.get_logger().warn(f"Prearm check failed - AP: {future.result().message}")


    def arm_request(self):
        if self.arming_in_progress or self.current_ap_status_armed:
            return

        self.get_logger().info("Starting arming attempt...")
        self.arming_in_progress = True

        req = ArmMotors.Request()
        req.arm = True # True to arm, False to disarm
        future = self._client_arm.call_async(req)
        future.add_done_callback(self._on_arm_complete)

    def _on_arm_complete(self, future):

        try:
            armed = future.result().result
        except Exception:
            armed = False

        self.arming_in_progress = False

        if armed:
            self.get_logger().info("Arming OK!")
            return

        self.get_logger().warn("Arming failed.")


    def switch_mode_request(self, mode):
        if self.mode_switch_in_progress:
            return

        self.get_logger().info(f"Starting mode switch attempt to mode {mode}...")
        self.mode_switch_in_progress = True

        req = ModeSwitch.Request()
        req.mode = mode
        future = self.mode_switch_client.call_async(req)
        future.add_done_callback(self._on_mode_switch_complete)

    def _on_mode_switch_complete(self, future):

        try:
            is_mode_ok = future.result().status
        except Exception:
            is_mode_ok = False

        self.mode_switch_in_progress = False

        if is_mode_ok:
            self.get_logger().info("Mode switch OK!")
            return

        self.get_logger().warn("Mode switch failed.")


    def takeoff_request(self, alt: float):
        if self.current_ap_status_flying or self.takeoff_in_progress:
            return
        
        self.get_logger().info(f"Requesting takeoff to {alt} meters.")
        self.takeoff_in_progress = True

        req = Takeoff.Request()
        req.alt = alt
        future = self.takeoff_client.call_async(req)
        future.add_done_callback(self._on_takeoff_complete)
        
    def _on_takeoff_complete(self, future):
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
            self.get_logger().warn("Takeoff request failed. Will allow retry.")
            self.takeoff_in_progress = False
            return

        # If request succeeded, leave takeoff_in_progress = True
        # until FCU reports that we are flying.


    """ --- Callbacks --- """

    # def _navsatfix_callback(self, msg: NavSatFix):
    #     # Process incomming NavSatFix topic messages

    #     stamp = msg.header.stamp
    #     if stamp.sec:
    #         self.get_logger().info(f"From AP : NavSatFix [sec:{stamp.sec}, nsec:{stamp.nanosec}]")
    #         self.curr_navsatfix_status = msg

    def _ap_status_callback(self, msg: Status):
        stamp = msg.header.stamp
        if stamp.sec:
            self.current_ap_status_mode = msg.mode
            self.current_ap_status_armed = msg.armed
            self.current_ap_status_flying = msg.flying


    """ --- Methods --- """

    def prepare_for_takeoff(self)
        # Wait for prearm
            if not self.is_prearm_ok:
                return

            if not self.mode_switch_in_progress and self.current_ap_status_mode != COPTER_MODE_GUIDED:
                self.switch_mode_request(COPTER_MODE_GUIDED)
                return
            elif self.mode_switch_in_progress:
                return

            # Start arming once prearm succeeds and mode switch completed
            if not self.arming_in_progress and not self.current_ap_status_armed:
                self.arm_request()
                return

            elif self.arming_in_progress:
                return

    """ --- Copter Control loop Methods --- """

    def copter_control_loop(self):

        try:
            
            if not self.current_ap_status_flying:

                self.prepare_for_takeoff()

                if not self.current_ap_status_flying or self.takeoff_in_progress:
                    self.takeoff_request(alt = TAKEOFF_ALT)

            elif self.current_ap_status_flying:
                pass 
                


        except Exception as e:
            self.get_logger().error(f"Copter control loop error: {e}")





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