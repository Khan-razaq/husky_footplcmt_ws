#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, tty, termios, signal

class JointTeleop(Node):
    def __init__(self):
        super().__init__('joint_teleop')
        self.publisher = self.create_publisher(
            Float64MultiArray, '/pole_position_controller/commands', 10)
        self.position = 0.0
        self.step = 0.1  # radians per keypress
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # Setup signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        print('\n\nKeyboard interrupt received. Cleaning up...')
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        return key
    
    def run(self):
        print("Use 'a'/'d' to rotate joint, 'q' to quit")
        print("Press Ctrl+C to exit cleanly\n")
        
        try:
            while True:
                key = self.get_key()
                if key == 'a':
                    self.position -= self.step
                elif key == 'd':
                    self.position += self.step
                elif key == 'q':
                    break
                
                # Clamp to joint limits
                self.position = max(-3.14, min(3.14, self.position))
                
                msg = Float64MultiArray()
                msg.data = [self.position]
                self.publisher.publish(msg)
                print(f"\rPosition: {self.position:.2f}", end='', flush=True)
        finally:
            self.cleanup()
            print('\nExiting...')

def main():
    rclpy.init()
    node = JointTeleop()
    try:
        node.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()