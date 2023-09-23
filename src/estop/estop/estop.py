from our_msgs.msg import EmergencyStop
import rclpy
from rclpy.node import Node
import tty
import select
import sys
import termios

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__("emergency_stop")
        self.pub = self.create_publisher(EmergencyStop, "emergency_stop_signal", 10)
        self.create_timer(.1, self.run_loop)
        self.msg = EmergencyStop()
        self.msg.should_stop = False
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run_loop(self):
        key = self.get_key()
        if key == "s":
            self.msg.should_stop = True
        if key == "\x03":
            raise KeyboardInterrupt
            
        self.pub.publish(self.msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()
    rclpy.spin(node)
    rclpy.shutdown()