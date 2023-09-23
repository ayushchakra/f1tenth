import rclpy
from rclpy.node import Node
from enum import Enum, auto
from our_msgs.msg import Control
from time import perf_counter

class BasicTrackControlsStates(Enum):
    FORWARD = auto()
    TURNING = auto()

class BasicTrackControlsNode(Node):
    def __init__(self):
        super().__init__("basic_track_controls")
        self.controls_pub = self.create_publisher(Control, "control_signal", 10)
        self.create_timer(.1, self.run_loop)
        self.state = BasicTrackControlsStates.FORWARD
        self.start_time = None
        
    def run_loop(self):
        if not self.start_time:
            self.start_time = perf_counter()
        if perf_counter() - self.start_time > 3:
            self.start_time = perf_counter()
            if self.state == BasicTrackControlsStates.FORWARD:
                self.state = BasicTrackControlsStates.TURNING
                self.controls_pub.publish(Control(velocity=0.3, steering_angle=3.14/6))
            else:
                self.state = BasicTrackControlsStates.FORWARD
                self.controls_pub.publish(Control(velocity=0.3, steering_angle=0.0))
                
                
def main(args=None):
    rclpy.init(args=args)
    node = BasicTrackControlsNode()
    rclpy.spin(node)
    rclpy.shutdown()
    