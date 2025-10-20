

# based on https://github.com/ArduPilot/ardupilot/blob/2110d5324f2a8f8cd1d1fe58c4f77ee58d9530ad/Tools/ros2/ardupilot_dds_tests/ardupilot_dds_tests/copter_takeoff.py 


# TODO vi kan vel stadig simme det her med en enkelt drone


import rclpy
import time

from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
# Ardupilot msg/srv: https://github.com/ArduPilot/ardupilot/tree/master/Tools/ros2/ardupilot_msgs:
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import Takeoff


CONTROL_LOOP_DT = 0.1  # seconds
TAKEOFF_ALT = 10.5 # feel free to change

COPTER_MODE_GUIDED = 4 # Dont change


class CopterTakeoff(Node):

    def __init__(self) -> None:
        """Initialise the node."""
        super().__init__("'copter_takeoff_and_land'")

        # Configure QoS profile for publishing and subscribing
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Create service clients
        self.arm_client = self.create_client(ArmMotors, "/ap/arm_motors")
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        self.mode_switch_client = self.create_client(ModeSwitch, "/ap/mode_switch")
        while not self.mode_switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')

        self.takeoff_client = self.create_client(Takeoff, "/ap/experimental/takeoff")
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available, waiting again...')

        # Create subscribers
        self.geopose_subscriber = self.create_subscription(GeoPoseStamped, "/ap/geopose/filtered", self.geopose_callback, qos_profile)

        # Create a timer to publish control commands
        self.timer = self.create_timer(CONTROL_LOOP_DT, self.control_loop_callback)

        # Initialize variables
        self.current_geopose = GeoPoseStamped()
        self.switched_to_guided = False
        self.arming_complete = False
        self.disarming_complete = False


    ##################### METHODS #####################

    def geopose_callback(self, msg: GeoPoseStamped): # https://docs.ros.org/en/noetic/api/geographic_msgs/html/msg/GeoPoseStamped.html
        """Process a GeoPose message."""
        stamp = msg.header.stamp
        if stamp.sec: # only process if we have a valid timestamp
            #self.get_logger().info("From AP : Geopose [sec:{}, nsec: {}]".format(stamp.sec, stamp.nanosec))

            # Store current geopose
            self.current_geopose = msg

    def arm(self):
        self.arming_complete = False
        req = ArmMotors.Request()
        req.arm = True
        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_client_callback)
        #rclpy.spin_until_future_complete(self, future) # TODO DER ER MÅSKE NOGET HER. DET HER ER BLOCKING HVIS VI KALDER DET I MAIN LOOP. SÅ SKAL arm_with_timeout nok også laves om i
        #return future.result()

    def arm_client_callback(self, future):
        result = future.result()
        # Set flag True if succesfully armed:
        self.arming_complete = result.result # (yes, there is a result field in ArmMotors.srv)


    def disarm(self):
        self.disarming_complete = False
        req = ArmMotors.Request()
        req.arm = False # false for "disarm"
        future = self.arm_client.call_async(req)
        future.add_done_callback(self.disarm_client_callback)


    def disarm_client_callback(self, future):
        result = future.result()
        # Set flag True if succesfully disarmed:
        self.disarming_complete = result.result # (yes, there is a result field in ArmMotors.srv)

    # def arm_with_timeout(self, timeout: rclpy.duration.Duration):
    #     # Try to arm. Returns true on success, or false if arming fails or times out.
    #     armed = False
    #     start = self.get_clock().now()
    #     while not armed and self.get_clock().now() - start < timeout:
    #         armed = self.arm().result
    #         time.sleep(1) #iTODO.. er det fint?? hvis vi har et main loop???
    #     return armed

    def switch_to_guided_mode(self):
        self.switched_to_guided = False # reset flag
        req = ModeSwitch.Request()
        req.mode = COPTER_MODE_GUIDED
        future = self.mode_switch_client.call_async(req)
        future.add_done_callback(self.switch_to_guided_mode_client_callback)
        #rclpy.spin_until_future_complete(self, future) # TODO samme som i arm (også se switch_to_guided_mode_with_timeout )
        #return future.result()

    def switch_to_guided_mode_client_callback(self, future):
        result = future.result() # TODO de havde future.result().result i deres kode.. men det virker forkert?? kom tilbage hvis det ikke virker

        # Set flag True if succesfully switched to guided mode:
        self.switched_to_guided = (result.status) or (result.curr_mode == COPTER_MODE_GUIDED)

    # def switch_to_guided_mode_with_timeout(self, timeout: rclpy.duration.Duration):
    #     # Try to switch to guided mode. Returns true on success, or false if it fails or times out.
    #     switched = False
    #     start = self.get_clock().now()
    #     while not switched and self.get_clock().now() - start < timeout:
    #         result = self.switch_to_guided_mode().result
    #         switched = (result.status) or (result.curr_mode == COPTER_MODE_GUIDED)
    #         time.sleep(1)
    #     return switched
    

 

    

    def takeoff(self, alt):
        req = Takeoff.Request()
        req.alt = alt
        future = self._client_takeoff.call_async(req)
        rclpy.spin_until_future_complete(self, future) # TODO
        return future.result()

    # def takeoff_with_timeout(self, alt: int, timeout: rclpy.duration.Duration):
    #     """Try to takeoff. Returns true on success, or false if takeoff fails or times out."""
    #     takeoff_success = False
    #     start = self.get_clock().now()
    #     while not takeoff_success and self.get_clock().now() - start < timeout:
    #         result = self.takeoff(alt)
    #         takeoff_success = result.status
    #         time.sleep(1) # TODO

    #     return takeoff_success

    # def get_cur_geopose(self): # TODO slet hvis ikke brugt
    #     """Return latest geopose."""
    #     return self.current_geopose
    
    ##################### MAIN CONTROL LOOP #####################

    def control_loop_callback(self) -> None:

        start_time = self.get_clock().now()

        # STEP 1: Switch to guided mode if not already in guided mode
        if not self.switched_to_guided:
            #self.get_logger().info("Switching to GUIDED mode...")
            self.switch_to_guided_mode()
            return  # wait for next loop iteration
        self.get_logger().info("Copter is in GUIDED mode.")

        # STEP 2: Arm motors if not already armed





        # ERROR CHECKING - Make sure control loop processing time does not exceed CONTROL_LOOP_DT:
        end_time = self.get_clock().now()
        elapsed_time_s = (end_time - start_time).nanoseconds / 1e9
        if elapsed_time_s > CONTROL_LOOP_DT:
            self.get_logger().warn(f"Control loop overrun: {elapsed_time_s:.4f} seconds")

        pass



def main(args=None) -> None:
    print('Starting copter takeooff example node...')
    rclpy.init(args=args)

    copter_takeoff = CopterTakeoff()
    rclpy.spin(copter_takeoff)

    copter_takeoff.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)