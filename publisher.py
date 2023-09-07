#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

#from example_interfaces.msg import Int64
from sensor_msgs.msg import Temperature
import random

class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__("temperature_sensor")
        self.temperature_publisher_ = self.create_publisher(
            Temperature, "temperature", 10)
        self.temperature_timer_ = self.create_timer(
            2.0, self.publish_temperature)

    def publish_temperature(self):
        temperature = 2.0
        msg = Temperature()
        msg.temperature = temperature
        self.temperature_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()