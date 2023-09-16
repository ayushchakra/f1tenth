import rclpy
from rclpy.node import Node
from our_msgs.msg import Control, CarState

import torch
from pytorch_mppi import MPPI


# TODO: Define dynamics:(state, control -> next_state)
#          and cost:(state, control -> cost)

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
            CarState,
            'state_estimate',
            self.run_controller_on_state_estimation_update,
            queue_size)

        self.subscription  # prevent unused variable warning


    def publish_control(self, control):
        """
        """
        msg = Control()
        msg.velocity       = control[0]
        msg.steering_angle = control[1]
        self.control_action_publisher.publish(msg)


    def unpack_state_from_msg(self, msg):
        state = [msg.pos_x, msg.pos_y, msg.yaw_angle, msg.velocity, msg.yaw_rate]
        return state


    def run_controller_on_state_estimation_update(self, msg):
        """
        """
        state = self.unpack_state_from_msg(msg)

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
