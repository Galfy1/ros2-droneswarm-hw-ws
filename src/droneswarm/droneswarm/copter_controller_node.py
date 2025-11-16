#!/usr/bin/env python3


import rclpy
import time

from rclpy.node import Node
from std_srvs.srv import Trigger
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import Takeoff


class CopterControllerNode(Node):

    def __init__(self):
        """ Initialize the copter controller node """
        super().__init__("copter_controller")

    
        # --- Create callback timers --- 
        self.copter_controll_loop_timer = self.create_timer(0.1, self.copter_control_loop)  # 10 Hz
        self.prearm_check_timer = self.create_timer(1.0, self.prearm_check)  # check every 1s
        # self.arm_attempt_timer = None

        # --- Services ---
        self._client_prearm = self.create_client(Trigger, "/ap/prearm_check")
        self._client_arm = self.create_client(ArmMotors, "/ap/arm_motors")
        # self.mode_client = self.create_client(ModeSwitch, "/ap/mode_switch")
        # self.takeoff_client = self.create_client(Takeoff, "/ap/experimental/takeoff")

        while not self._client_prearm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('prearm service not available, waiting again...')


        
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

#         self.mode_switch_client = self.create_client(ModeSwitch, "/ap/mode_switch")
#         while not self.mode_switch_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('takeoff service not available, waiting again...')

#         self.takeoff_client = self.create_client(Takeoff, "/ap/experimental/takeoff")
#         while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('takeoff service not available, waiting again...')


        # Configure QoS profile for publishing and subscribing
#         qos_profile = rclpy.qos.QoSProfile(
#             reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
#             durability=rclpy.qos.DurabilityPolicy.VOLATILE
#         )


        # Initializing copter controller flags
        self.prearm_check_in_progress = False
        self.is_prearm_ok = False
        self.arming_in_progress = False
        self.is_armed_ok = False


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

        self.get_logger().warn("Prearm attempt failed. Trying again...")


    # def start_arming(self):
    #     if self.is_armed_ok:
    #         return

    #     self.get_logger().info("Starting arming attempts...")

    #     # Retry arming once per second
    #     self.arm_attempt_timer = self.create_timer(1.0, self.arm_request)

    def arm_request(self):
        if self.arming_in_progress or self.is_armed_ok:
            return
        print("Sending arm request...")
        self.arming_in_progress = True

        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        future.add_done_callback(self._on_arm_complete)

    def _on_arm_complete(self, future):

        # If we already armed, ignore late results
        if self.is_armed_ok:
            return
        try:
            self.is_armed_ok = future.result().result
        except Exception:
            self.is_armed_ok = False

        self.arming_in_progress = False

        if self.is_armed_ok:
            self.get_logger().info("Arming OK! Stopping arming attempts.")
            return

        self.get_logger().warn("Arming failed. Retrying...")


    """ --- Methods --- """



    """ --- Copter Control loop Methods --- """

    def copter_control_loop(self):

        # Wait for prearm
        if not self.is_prearm_ok:
            return

        # Start arming once prearm succeeds
        if not self.is_armed_ok and not self.arming_in_progress:
            self.arm_request()
            return
        elif self.arming_in_progress:
            return

        # Fully prearmed + armed
        self.get_logger().info("Control loop: Copter is armed")




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