import rclpy
from rclpy.node import Node
from mppi_control.msg import Control
#from state_estimation.msg import State

from pytorch_mppi import MPPI
import torch


class MPPI_Node(Node):

    def __init__(self, dynamics, running_cost, nx, noise_sigma, N_SAMPLES, TIMESTEPS, lambda_, ACTION_LOW, ACTION_HIGH, d='cpu'):
        queue_size = 10
        # create controller with chosen parameters
        self.controller = MPPI(dynamics, running_cost, nx, noise_sigma, num_samples=N_SAMPLES, horizon=TIMESTEPS,
                          lambda_=lambda_, device=d,
                          u_min=torch.tensor(ACTION_LOW, dtype=torch.double, device=d),
                          u_max=torch.tensor(ACTION_HIGH, dtype=torch.double, device=d))

        #super().__init__('minimal_publisher')

        self.control_action_publisher = self.create_publisher(
            Control,
            'control_input',
            queue_size)

        self.state_subscription = self.create_subscription(
            State,
            'state_estimate',
            self.run_controller_on_state_estimation_update,
            queue_size)

        self.subscription  # prevent unused variable warning


    def publish_control(self, control):
        """
        """
        msg = Control()
        msg.data = 'Hello World'   # control...
        self.control_action_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


    def unpack_state_from_msg(self, data):
        pass
        #state = f(data)...


    def run_controller_on_state_estimation_update(self, msg):
        """
        """
        self.get_logger().info('Received state estimate: "%s"' % msg.data)

        state = self.unpack_state_from_msg(msg.data)

        # Call controller
        control_action = self.controller.command(state)

        self.publish_control(control_action)



def main(args=None):
    rclpy.init(args=args)

    mppi_node = MPPI_Node(...)

    rclpy.spin(mppi_node)

    mppi_node.destroy_node()        # Destroy the node explicitly
    rclpy.shutdown()


if __name__ == '__main__':
    main()
