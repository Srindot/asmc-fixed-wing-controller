#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import ActuatorServos
import time
import math

class InspectActuatorServos(Node):
    """Node to inspect the ActuatorServos message structure."""

    def __init__(self) -> None:
        super().__init__('inspect_actuator_servos')
        self.timer = self.create_timer(1.0, self.timer_callback)  # Check once per second

    def timer_callback(self) -> None:
        msg = ActuatorServos()
        fields_and_types = msg.get_fields_and_field_types()
        self.get_logger().info("Available fields and their types in ActuatorServos:")
        for field, type_name in fields_and_types.items():
            self.get_logger().info(f"  {field}: {type_name}")
        self.destroy_timer(self.timer)  # Only run once
        rclpy.shutdown()

def main(args=None) -> None:
    print('Starting node to inspect ActuatorServos message...')
    rclpy.init(args=args)
    inspect_node = InspectActuatorServos()
    rclpy.spin(inspect_node)
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)