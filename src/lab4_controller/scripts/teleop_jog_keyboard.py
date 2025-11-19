#!/usr/bin/env python3

"""
Teleoperation Keyboard Node for 3R Robot Control
Publishes Twist messages to /cmd_vel topic for velocity control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class TeleopKeyboard(Node):
    """
    Node for keyboard teleoperation of the 3R robot.
    Controls End-Effector velocity through keyboard inputs.
    """
    
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Velocity parameters
        self.linear_speed = 0.1   # m/s
        self.speed_increment = 0.02  # m/s
        
        # Key bindings
        self.key_bindings = {
            # Linear motion
            'w': (1, 0, 0),   # Forward (+X)
            's': (-1, 0, 0),  # Backward (-X)
            'a': (0, 1, 0),   # Left (+Y)
            'd': (0, -1, 0),  # Right (-Y)
            'q': (0, 0, 1),   # Up (+Z)
            'e': (0, 0, -1),  # Down (-Z)
            
            # Combined motions
            'r': (1, 1, 0),   # Forward-Left
            't': (1, -1, 0),  # Forward-Right
            'f': (-1, 1, 0),  # Backward-Left
            'g': (-1, -1, 0), # Backward-Right
        }
        
        # Speed control
        self.speed_bindings = {
            '+': 1.1,  # Increase speed by 10%
            '-': 0.9,  # Decrease speed by 10%
        }
        
        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Teleoperation Keyboard Node Started')
        self.print_instructions()
        
    def print_instructions(self):
        """Print usage instructions to terminal."""
        msg = """

3R Robot Teleoperation Keyboard Control      
---------------------------------------------------   
  Movement Controls (End-Effector Velocity):             
    W: Forward (+X)       Q: Up (+Z)                   
    S: Backward (-X)      E: Down (-Z)                  
    A: Left (+Y)                                        
    D: Right (-Y)                                       
---------------------------------------------------                                                             
  Combined Motions:                                       
    R: Forward-Left       T: Forward-Right              
    F: Backward-Left      G: Backward-Right             
---------------------------------------------------                                                             
  Speed Control:                                          
    +: Increase speed by 10%                          
    -: Decrease speed by 10%                          
---------------------------------------------------                                                             
  Other:                                                 
    SPACE: Stop all motion                             
    Ctrl+C: Exit                                       
---------------------------------------------------                                                            
  Current Speed: {:.3f} m/s                           
    """.format(self.linear_speed)
        print(msg)
        print("NOTE: Make sure you activate TO_F or TO_G mode in scheduler first!")
        print("      TO_F = End-Effector Frame")
        print("      TO_G = Global/World Frame")
        print("="*60)
        
    def get_key(self, timeout=0.1):

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            # Handle special characters
            if key == '\x03':  # Ctrl+C
                raise KeyboardInterrupt
            return key
        return None
    
    def publish_velocity(self, linear_x, linear_y, linear_z):

        msg = Twist()
        msg.linear.x = linear_x * self.linear_speed
        msg.linear.y = linear_y * self.linear_speed
        msg.linear.z = linear_z * self.linear_speed
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(msg)
        
    def publish_stop(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        
    def run(self):

        try:
            while True:
                key = self.get_key()
                
                if key is None:
                    # No key pressed - publish zero velocity
                    self.publish_stop()
                    continue
                
                # Check for movement commands
                if key in self.key_bindings:
                    vx, vy, vz = self.key_bindings[key]
                    self.publish_velocity(vx, vy, vz)
                    
                    # Show current command
                    self.get_logger().info(
                        f'Moving: X={vx*self.linear_speed:.3f}, '
                        f'Y={vy*self.linear_speed:.3f}, '
                        f'Z={vz*self.linear_speed:.3f} m/s'
                    )
                
                # Check for speed adjustment
                elif key in self.speed_bindings:
                    self.linear_speed *= self.speed_bindings[key]
                    self.linear_speed = max(0.01, min(1.0, self.linear_speed))  # Limit speed
                    self.get_logger().info(f'Speed adjusted to: {self.linear_speed:.3f} m/s')
                
                # Space to stop
                elif key == ' ':
                    self.publish_stop()
                    self.get_logger().info('Emergency stop!')
                
                # Unknown key
                else:
                    if key.isprintable():
                        self.get_logger().warn(f'Unknown key: {key}')
                        
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt received. Shutting down...')
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # Send stop command before exit
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TeleopKeyboard()
        node.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()