#!/usr/bin/env python3


import rclpy
import time

from rclpy.node import Node
from std_srvs.srv import Trigger


class CopterControllerNode(Node):

    def __init__(self):
        """ Initialize the copter controller node """
        super().__init__("copter_controller")

    
        # --- Create callback timers --- 
        self.copter_controll_loop_timer = self.create_timer(0.1, self.copter_control_loop)  # 10 Hz
        self.prearm_check_timer = self.create_timer(1.0, self.prearm_check)  # check every 1s

        # --- Services ---
        self._client_prearm = self.create_client(Trigger, "/ap/prearm_check")
        # self.arm_client = self.create_client(ArmMotors, "/ap/arm_motors")
        # self.mode_client = self.create_client(ModeSwitch, "/ap/mode_switch")
        # self.takeoff_client = self.create_client(Takeoff, "/ap/experimental/takeoff")

        while not self._client_prearm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('prearm service not available, waiting again...')


        # Configure QoS profile for publishing and subscribing
#         qos_profile = rclpy.qos.QoSProfile(
#             reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
#             durability=rclpy.qos.DurabilityPolicy.VOLATILE,
#             depth=1
#         )

        # Initializing copter controller flags
        self.prearm_check_in_progress = False
        self.is_prearm_ok = False


    """ --- Service Methods  --- """

    def prearm_check(self):
        if self.prearm_check_in_progress:
            return

        self.prearm_check_in_progress = True

        req = Trigger.Request()
        future = self._client_prearm.call_async(req)
        future.add_done_callback(self._on_prearm_check_complete)

    def _on_prearm_check_complete(self, future):

        self.is_prearm_ok = future.result().success
        self.prearm_check_in_progress = False

        # Stop checking if success
        if self.is_prearm_ok:
            self.get_logger().info("Prearm OK. Stopping prearm checks.")
            self.prearm_check_timer.cancel()
            return
        else:
            self.get_logger().info("Prearm attempt failed. Trying again...")
            return


    """ --- Methods --- """



    """ --- Copter Control loop Methods --- """

    def copter_control_loop(self):
        # TODO: main control loop of the drone

        if not self.is_prearm_ok:
            return

        self.get_logger().info("Control loop, prearm check complete!")



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