#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class SimpleJointController(Node):
    def __init__(self):
        super().__init__('simple_joint_controller')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.get_logger().info("Simple Joint Controller started. Type target angles (in degrees):")

    def set_joint_position(self, angle_degrees):
        angle_radians = angle_degrees * 3.14159 / 180.0
        
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['base_to_pole']
        msg.position = [angle_radians]
        msg.velocity = [0.0]
        msg.effort = []
        
        # Publish multiple times to ensure it's received
        for i in range(10):
            self.joint_pub.publish(msg)
            time.sleep(0.1)
        
        self.get_logger().info(f"Set joint to {angle_degrees} degrees ({angle_radians:.2f} radians)")

def main():
    rclpy.init()
    controller = SimpleJointController()
    
    try:
        while rclpy.ok():
            try:
                angle = float(input("Enter target angle (degrees): "))
                controller.set_joint_position(angle)
            except ValueError:
                print("Please enter a valid number")
            except KeyboardInterrupt:
                break
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()