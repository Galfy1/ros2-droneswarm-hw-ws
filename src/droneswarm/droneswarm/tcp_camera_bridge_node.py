#! /usr/bin/env python3

import json
import rclpy 
import threading
from rclpy.node import Node

from droneswarm.tcp_server import TCPServer
from our_custom_interfaces.msg import ObjectData



class TCPCameraBridgeNode(Node):
    def __init__(self, host="localhost", port=5050) -> None:
        super().__init__('tcp_camera_bridge_node')
        self.get_logger().info("TCP Camera Bridge Node started.")

        self.publisher = self.create_publisher(ObjectData, '/detections', 10)

        self.tcp_server = TCPServer(host=host, port=port)
        self.get_logger().info(f"Connecting to TCP server at {host}:{port}...")
        self.tcp_server.connect()
        self.get_logger().info("Connected")

        self._running = True

        self.listener_thread = threading.Thread(target=self.tcp_listener, daemon=True)
        self.listener_thread.start()


    def tcp_listener(self) -> None:
        """ Listen for incoming TCP messages and publish them as ROS2 messages """

        while self._running:
            try:
                msg_json_str = self.tcp_server.receive()
                if msg_json_str:
                    msg_data = json.loads(msg_json_str)
                
                    # Convert incoming message to ROS2 message
                    ros_msg = ObjectData()
                    msg_data_box = msg_data.get('box')
                    if msg_data_box:
                        ros_msg.x = msg_data_box[0]
                        ros_msg.y = msg_data_box[1]
                        ros_msg.w = msg_data_box[2]
                        ros_msg.h = msg_data_box[3]
                    ros_msg.category = msg_data.get('category')
                    ros_msg.confidence = msg_data.get('confidence')
                    ros_msg.err_x = msg_data.get('err_x')
                    ros_msg.err_y = msg_data.get('err_y')

                    self.publisher.publish(ros_msg) # Publish the ROS2 message
                    
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
                self._running = False

    def destroy_node(self) -> None:
        self.get_logger().info("Shutting down TCP Camera Bridge Node...")
        self._running = False
        self.tcp_server.stop_server()
        super().destroy_node()
        

def main(args=None):
    print('Starting TCP Camera Bridge Node...')
    rclpy.init(args=args)

    tcp_camera_bridge_node = TCPCameraBridgeNode(host="localhost", port=5050)
    rclpy.spin(tcp_camera_bridge_node)

    tcp_camera_bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()