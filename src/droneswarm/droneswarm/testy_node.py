import rclpy
import time

from rclpy.node import Node
# from geographic_msgs.msg import GeoPoseStamped
# from ardupilot_msgs.srv import ArmMotors
# from ardupilot_msgs.srv import ModeSwitch
# from ardupilot_msgs.srv import Takeoff

from std_msgs.msg import String


class TestyNode(Node):

    def __init__(self):
        """Initialise the node."""
        super().__init__("testy_node")
        self.get_logger().info("Hello from Testy Node!")
        
        # just a whatever topic (so we can see using ros2 topic list that the node is running)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.cmd_gps_pose_publisher = self.create_publisher(String, "ap/cmd_gps_pose", qos_profile)

        self.timer = self.create_timer(1, self.control_loop_callback)

    def control_loop_callback(self):
        self.get_logger().info("Testy Node is running...")

def main(args=None) -> None:
    print('Starting Testy Node...')
    rclpy.init(args=args)

    testy_node = TestyNode()
    rclpy.spin(testy_node)

    testy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)