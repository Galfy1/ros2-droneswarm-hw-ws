import rclpy
import time

from rclpy.node import Node
# from geographic_msgs.msg import GeoPoseStamped
# from ardupilot_msgs.srv import ArmMotors
# from ardupilot_msgs.srv import ModeSwitch
# from ardupilot_msgs.srv import Takeoff



class TestyNode(Node):

    def __init__(self):
        """Initialise the node."""
        super().__init__("testy_node")
        self.get_logger().info("Hello from Testy Node!")



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