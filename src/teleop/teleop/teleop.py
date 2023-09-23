import rclpy
from rclpy.node import Node
from our_msgs.msg import Control
import tty
import select
import sys
import termios
from .controller import LogitechController
import numpy as np

CONTROLLER_MODE = "KEYBOARD"    # JOYSTICK or KEYBOARD

class SendControl(Node):
    key_to_vel = {
        "s": Control(
            steering_angle=0.0, velocity=0.0
        ),
        "q": Control(
            steering_angle=0.349, velocity=.5
        ),
        "w": Control(
            steering_angle=0.0, velocity=.5
        ),
        "a": Control(
            steering_angle=0.349, velocity=.5
        ),
        "z": Control(
            steering_angle=0.349, velocity=-.5
        ),
        "x": Control(
            steering_angle=0.349, velocity=-.5
        ),
        "c": Control(
            steering_angle=-0.349, velocity=-.5
        ),
    }

    def __init__(self):
        super().__init__('send_drive_command')
        if CONTROLLER_MODE == "KEYBOARD":
            self.settings = termios.tcgetattr(sys.stdin)
        else:
            self.controller = LogitechController()
        self.drive_command_pub = self.create_publisher(Control, "control_signal", 10)
        self.timer = self.create_timer(.1, self.run_loop)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run_loop(self):
        if CONTROLLER_MODE == "KEYBOARD":
            self.key = self.get_key()
            if self.key == "\x03":
                self.drive_command_pub.publish(self.key_to_vel["s"])
                raise KeyboardInterrupt
            if self.key in self.key_to_vel.keys():
                self.drive_command_pub.publish(self.key_to_vel[self.key])
        else:
            self.controller.listen()

            controller_x = self.controller.axis_data[3]
            drive_velocity = float((-self.controller.axis_data[4]))*1.4
            drive_angle = -np.deg2rad(controller_x*35)
            self.drive_command_pub.publish(Control(steering_angle=drive_angle, velocity=drive_velocity))

def main(args=None):
    rclpy.init()
    node = SendControl()
    rclpy.spin(node)
    rclpy.shutdown()
    