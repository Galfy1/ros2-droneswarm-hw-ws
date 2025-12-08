#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from our_custom_interfaces.msg import ObjectData

def generate_dataset():
    samples = []

    # 50 invalid samples at start
    for _ in range(50):
        samples.append({
            "valid": False,
            "x": 0,
            "y": 0,
            "w": 0,
            "h": 0,
            "category": 0,
            "confidence": 0.0,
            "err_x": 0,
            "err_y": 0
        })

    sequences = [
        ("err_x", 320, 0),
        ("err_x", -320, 0),
        ("err_y", 240, 0),
        ("err_y", -240, 0),
        ("w", 300, 100),
        ("w", 20, 100),
    ]

    # 6 sequences × 30 valid samples
    for field, start, end in sequences:
        for step in range(30):

            # linear inclusive interpolation
            v = start + (end - start) * step / 29

            # convert to int (only confidence remains float)
            v_int = int(round(v))

            sample = {
                "valid": True,
                "x": 0,
                "y": 0,
                "w": 100,
                "h": 100,
                "category": 0,
                "confidence": 0.0,
                "err_x": 0,
                "err_y": 0
            }

            if field in ["w", "h"]:
                sample["w"] = v_int
                sample["h"] = v_int
            else:
                sample[field] = v_int

            samples.append(sample)

    # 30 invalid samples at end
    for _ in range(30):
        samples.append({
            "valid": False,
            "x": 0,
            "y": 0,
            "w": 0,
            "h": 0,
            "category": 0,
            "confidence": 0.0,
            "err_x": 0,
            "err_y": 0
        })

    return samples


class DetectionGeneratorNode(Node):
    def __init__(self) -> None:
        super().__init__('detection_generator_node')
        self.get_logger().info("Detection Generator Node started.")

        self.publisher = self.create_publisher(ObjectData, '/detections', 10)

        self.data = generate_dataset()
        self.index = 0
        self.is_publishing = False

        self.timer = self.create_timer(0.1, self.publish_detection)  # 10 Hz

    def publish_detection(self) -> None:
        """ Publish the next detection in the dataset """

        if not self.is_publishing:
            return

        if self.index >= len(self.data):
            self.get_logger().info("All detections published.")
            rclpy.shutdown()
            return

        sample = self.data[self.index]
        ros_msg = ObjectData()
        ros_msg.valid = sample["valid"]
        ros_msg.x = sample["x"]
        ros_msg.y = sample["y"]
        ros_msg.w = sample["w"]
        ros_msg.h = sample["h"]
        ros_msg.category = sample["category"]
        ros_msg.confidence = sample["confidence"]
        ros_msg.err_x = sample["err_x"]
        ros_msg.err_y = sample["err_y"]

        self.publisher.publish(ros_msg)
        self.get_logger().info(f"Published detection {self.index + 1}/{len(self.data)}")

        self.index += 1


def main(args=None):
    print('Starting Detection Generator Node...')
    rclpy.init(args=args)

    detection_generator_node = DetectionGeneratorNode()
    
    user_input = input("Type 'yes' to start processing the dataset: ")
    if user_input.lower() == "yes":
        detection_generator_node.is_publishing = True
    else:
        print("Publishing not started.")
        detection_generator_node.destroy_node()
        rclpy.shutdown()
        return
    
    rclpy.spin(detection_generator_node)

    detection_generator_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()