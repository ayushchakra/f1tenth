import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
from board import SCL, SDA
import busio

from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

from our_msgs.msg import Control


class CarDriverNode(Node):
    def __init__(self):
        super().__init__("car_driver_node")

        self.control_sub = self.create_subscription(
            Control, "control_signal", self.set_control, 10
        )
        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c)
        print(self.pca.frequency)
        self.pca.frequency = 50 # TODO find out what this does
        self.steer_servo = servo.Servo(self.pca.channels[0], actuation_range=79, min_pulse=1080, max_pulse=1870)
        # self.esc_pwm = self.pca.channels[1] # TODO Convert ESC to real PWMOut
        self.esc = servo.ContinuousServo(self.pca.channels[1], min_pulse=1000, max_pulse=2000)

        self.STEER_ANGLE_OFFSET = 47
        self.TIRE_DIAMETER: int = 0.11  # m
        self.GEAR_RATIO: int = 42
        self.MAX_RAW_RPM: int = 10400
        self.MAX_VEL: float = ((self.MAX_RAW_RPM / self.GEAR_RATIO) / 60) * (
            (self.TIRE_DIAMETER * np.pi)
        )
        print(self.MAX_VEL)
        # self.calibrate_esc()
        # self.arm_esc()

    def set_control(self, msg: Control):
        self.set_drive_velocity(msg.velocity)
        self.set_steering_angle(msg.steering_angle)

    def set_steering_angle(self, angle: float):
        self.steer_servo.angle = np.clip(math.degrees(angle) + self.STEER_ANGLE_OFFSET,0,self.steer_servo.actuation_range)
        self.get_logger().info(f"Steer Angle {self.steer_servo.angle}")

    def set_drive_velocity(self, vel: float):
        self.esc.throttle = -vel / self.MAX_VEL
        # self.get_logger().info(f"ESC Throttle {self.esc.throttle}")

    def arm_esc(self):
        """
        This prompts the user to connect power to the ESC. This is to prepare
        the ESC to be calibrated.
        """
        self.esc.throttle = -1
        self.get_logger().info("Connect to power in the next 3 seconds...")
        time.sleep(1)
        self.get_logger().info("Connect to power in 3 seconds...")
        time.sleep(1)
        self.get_logger().info("Connect to power in 2 seconds...")
        time.sleep(1)
        self.get_logger().info("Connect to power in 1 seconds...")
        time.sleep(1)
        self.esc.throttle = -1
        time.sleep(4)
        self.get_logger().info("Arming Complete")
        self.esc.throttle = 0

    def calibrate_esc(self):
        """
        Calibration of the ESC by sending the minimum and maximum width for
        PWM inputs to the ESC.
        """
        self.esc.throttle = 1
        input("Connect power and press Enter to continue...")
        self.esc.throttle = 1
        time.sleep(2)
        self.esc.throttle = -1
        time.sleep(4)
        self.get_logger().info("Calibration Complete")

    def steer_servo_off(self):
        self.steer_servo.angle = None


def main(args=None):
    """
    Main function to run teleop node.
    """
    rclpy.init(args=args)
    node = CarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_drive_velocity(0)
        node.steer_servo_off()

    rclpy.shutdown()
