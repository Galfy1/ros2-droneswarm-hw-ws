#! /usr/bin/env python3

import rclpy 
from rclpy.node import Node



class TCPCameraBridgeNode(Node):
    def __init__(self):
        super().__init__('tcp_camera_bridge_node')
        self.get_logger().info("TCP Camera Bridge Node started.")


        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.get_logger().info('Publishing: "Hello World: %d"' % self.i)
        self.i += 1
        

def main(args=None):
    print('Starting TCP Camera Bridge Node...')
    rclpy.init(args=args)

    tcp_camera_bridge_node = TCPCameraBridgeNode()
    rclpy.spin(tcp_camera_bridge_node)

    tcp_camera_bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()