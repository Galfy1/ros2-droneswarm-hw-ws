

# based on https://github.com/ArduPilot/ardupilot/blob/2110d5324f2a8f8cd1d1fe58c4f77ee58d9530ad/Tools/ros2/ardupilot_dds_tests/ardupilot_dds_tests/copter_takeoff.py 


# TODO vi kan vel stadig simme det her med en enkelt drone


import rclpy
import time

from rclpy.node import Node
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Twist

from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from ardupilot_msgs.srv import Takeoff
from ardupilot_msgs.msg import GlobalPosition

# Ardupilot msg/srv interfaces: https://github.com/ArduPilot/ardupilot/tree/master/Tools/ros2/ardupilot_msgs
# Ardupilot ROS2 interfaces: https://ardupilot.org/dev/docs/ros2-interfaces.html 


CONTROL_LOOP_DT = 0.1  # seconds
TAKEOFF_ALT = 10.5 # feel free to change

COPTER_MODE_GUIDED = 4 # Dont change



def takeoff_land_loop(self):
    
    if self.current_geopose.pose.position.altitude < TAKEOFF_ALT:
        # Takeoff phase
        self.publish_position_setpoint_global(
            lat=self.current_geopose.pose.position.latitude,
            lon=self.current_geopose.pose.position.longitude,
            alt=TAKEOFF_ALT,
            velocity=1.0,
            yaw=0.0
        )
    else:
        # Land phase
        pass
        # TODO self.land()


class CopterTakeoff(Node):

    def __init__(self) -> None:
        """Initialise the node."""
        super().__init__("copter_takeoff_and_land")

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

        # Create publishers
        self.cmd_gps_pose_publisher = self.create_publisher(GlobalPosition, "ap/cmd_gps_pose", qos_profile)
    

        # Create a timer to publish control commands
        self.timer = self.create_timer(CONTROL_LOOP_DT, self.control_loop_callback)

        # Initialize variables
        self.current_geopose = GeoPoseStamped()
        self.switched_to_guided = False
        self.arming_complete = False
        self.disarming_complete = False
        self.ardupilot_takeoff_complete = False


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

    def switch_to_guided_mode(self):
        self.switched_to_guided = False # reset flag
        req = ModeSwitch.Request()
        req.mode = COPTER_MODE_GUIDED
        future = self.mode_switch_client.call_async(req)
        future.add_done_callback(self.switch_to_guided_mode_client_callback)

    def switch_to_guided_mode_client_callback(self, future):
        result = future.result() # TODO de havde future.result().result i deres kode.. men det virker forkert?? kom tilbage hvis det ikke virker

        # Set flag True if succesfully switched to guided mode:
        self.switched_to_guided = (result.status) or (result.curr_mode == COPTER_MODE_GUIDED)

    # def ardupilot_takeoff(self, alt):
    #     self.ardupilot_takeoff_complete = False
    #     req = Takeoff.Request()
    #     req.alt = alt
    #     future = self.takeoff_client.call_async(req)
    #     future = self.add_done_callback(self.ardupilot_takeoff_client_callback)

    def ardupilot_takeoff(self, alt: float):
        self.ardupilot_takeoff_complete = False
        req = Takeoff.Request()
        req.alt = float(alt)
        future = self.takeoff_client.call_async(req)
        future.add_done_callback(self.ardupilot_takeoff_client_callback)

    def land(self):
        pass
        # TODO skal gøres ved at sætte den i "land" mode.. aka switch_to_guided_mode skal nok laves om til at kunne tage en mode parameter

    def ardupilot_takeoff_client_callback(self, future):
        result = future.result()
        # Set flag True if succesfully took off:
        self.ardupilot_takeoff_complete = result.status

 
    def publish_position_setpoint_global(self, lat: float, lon: float, alt: float, velocity: float = 1.0, yaw: float = 0.0):
        # TODO THE ALT SIGN IS REVERSED COMPARED TO PX4

        msg = GlobalPosition()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.yaw = yaw
        velocity = Twist()
        velocity.linear.x = velocity
        velocity.linear.y = velocity
        velocity.linear.z = velocity
        msg.velocity = velocity
        self.cmd_gps_pose_publisher.publish(msg)
    
    
    ##################### MAIN CONTROL LOOP #####################

    def control_loop_callback(self) -> None:

        start_time = self.get_clock().now()

        # STEP 1: Switch to guided mode if not already in guided mode
        if not self.switched_to_guided:
            #self.get_logger().info("Switching to GUIDED mode...")
            self.switch_to_guided_mode()
            return  # wait for next loop iteration
        #self.get_logger().info("Copter is in GUIDED mode.")

        # STEP 2: Arm motors if not already armed
        if not self.arming_complete:
            #self.get_logger().info("Arming motors...")
            self.arm()
            return  # wait for next loop iteration
        
        # Step 3: Do initial "ardupilot takeoff"
        # (Not sure if ardupilot require using its "takeoff" service, before we can publish pos setpoints - or we if can publish pos directly after arming like on PX4
        # Just to make sure, we run an initial "ardupilot takeoff" command, to get in the air.)
        if not self.ardupilot_takeoff_complete:
            self.ardupilot_takeoff(5.0) # 1 meter
            return
    
        # STEP 4: do whatever you want in your "application":
        takeoff_land_loop(self)

        # ERROR CHECKING - Make sure control loop processing time does not exceed CONTROL_LOOP_DT:
        end_time = self.get_clock().now()
        elapsed_time_s = (end_time - start_time).nanoseconds / 1e9
        if elapsed_time_s > CONTROL_LOOP_DT:
            self.get_logger().warn(f"Control loop overrun: {elapsed_time_s:.4f} seconds")




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