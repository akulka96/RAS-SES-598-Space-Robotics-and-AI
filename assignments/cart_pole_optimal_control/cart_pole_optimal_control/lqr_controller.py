#!/usr/bin/env python3

import time
import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr_controller')

        # System parameters
        self.M = 1.0      # Mass of the cart (kg)
        self.m = 1.0      # Mass of the pole (kg)
        self.L = 1.0      # Length of the pole (m)
        self.g = 9.81     # Gravitational acceleration (m/s^2)

        # State space matrices
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ])
        self.B = np.array([
            [0],
            [1 / self.M],
            [0],
            [-1 / (self.M * self.L)]
        ])

        # LQR cost matrices
        self.Q = np.diag([7.0, 2.0, 3.0, 1.0])
        self.R = np.array([[0.01]])

        # Compute LQR gain matrix and log it
        self.K = self.compute_lqr_gain()
        self.get_logger().info(f'LQR Gain Matrix: {self.K}')

        # Initialize state and logging variables
        self.x = np.zeros((4, 1))
        self.state_initialized = False
        self.last_control = 0.0
        self.control_count = 0
        self.log_data = []
        self.start_time = time.time()

        # Set up publishers and subscribers
        self.cart_cmd_pub = self.create_publisher(
            Float64, '/model/cart_pole/joint/cart_to_base/cmd_force', 10
        )
        if self.cart_cmd_pub:
            self.get_logger().info('Force command publisher created successfully')

        self.joint_state_sub = self.create_subscription(
            JointState, '/world/empty/model/cart_pole/joint_state', self.joint_state_callback, 10
        )

        # Start control loop timer
        self.timer = self.create_timer(0.01, self.control_loop)
        self.get_logger().info('Cart-Pole LQR Controller initialized')

    def compute_lqr_gain(self):
        """
        Compute the LQR gain matrix K using the continuous-time Algebraic Riccati Equation.
        """
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K

    def joint_state_callback(self, msg):
        """
        Update the state estimate based on joint state message data.
        The expected state vector is [x, x_dot, theta, theta_dot].
        """
        try:
            # Identify indices for cart and pole joints
            cart_index = msg.name.index('cart_to_base')
            pole_index = msg.name.index('pole_joint')

            # Update the state vector
            self.x = np.array([
                [msg.position[cart_index]],
                [msg.velocity[cart_index]],
                [msg.position[pole_index]],
                [msg.velocity[pole_index]]
            ])

            if not self.state_initialized:
                self.get_logger().info(
                    f'Initial state: cart_pos={msg.position[cart_index]:.3f}, '
                    f'cart_vel={msg.velocity[cart_index]:.3f}, '
                    f'pole_angle={msg.position[pole_index]:.3f}, '
                    f'pole_vel={msg.velocity[pole_index]:.3f}'
                )
                self.state_initialized = True

        except (ValueError, IndexError) as error:
            self.get_logger().warn(f'Failed to process joint states: {error}, msg={msg.name}')

    def control_loop(self):
        """
        Compute the LQR control input and publish the force command.
        """
        if not self.state_initialized:
            self.get_logger().warn('State not initialized yet')
            return

        try:
            # LQR control law: u = -K * x
            control_input = -self.K @ self.x
            force = float(control_input[0])

            # Log control information periodically
            if abs(force - self.last_control) > 0.1 or self.control_count % 100 == 0:
                self.get_logger().info(f'State: {self.x.T}, Control force: {force:.3f}N')

            # Publish the control force
            msg = Float64()
            msg.data = force
            self.cart_cmd_pub.publish(msg)

            # Log data for later analysis
            elapsed_time = time.time() - self.start_time
            self.log_data.append([
                elapsed_time,
                self.x[0, 0],
                self.x[1, 0],
                self.x[2, 0],
                self.x[3, 0],
                force
            ])

            self.last_control = force
            self.control_count += 1

        except Exception as error:
            self.get_logger().error(f'Control loop error: {error}')

    def save_and_plot_data(self):
        """
        Save the logged performance data to a CSV file and generate performance plots.
        """
        with open('lqr_performance.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['time', 'x', 'x_dot', 'theta', 'theta_dot', 'force'])
            writer.writerows(self.log_data)

        # Convert log data to a numpy array for plotting
        data = np.array(self.log_data)
        t = data[:, 0]
        x = data[:, 1]
        theta = data[:, 3]
        force = data[:, 5]

        # Plot cart position
        plt.figure(figsize=(12, 8))
        plt.subplot(3, 1, 1)
        plt.plot(t, x, label='Cart Position (m)')
        plt.axhline(y=0.5, color='r', linestyle='--', label='Safe Limit')
        plt.axhline(y=-0.5, color='r', linestyle='--')
        plt.ylabel('Cart Position (m)')
        plt.legend()

        # Plot pendulum angle in degrees
        plt.subplot(3, 1, 2)
        plt.plot(t, theta * 180 / np.pi, label='Pendulum Angle (°)')
        plt.axhline(y=3, color='r', linestyle='--', label='Stability Threshold')
        plt.axhline(y=-3, color='r', linestyle='--')
        plt.ylabel('Pendulum Angle (°)')
        plt.legend()

        # Plot control force
        plt.subplot(3, 1, 3)
        plt.plot(t, force, label='Control Force (N)')
        plt.ylabel('Control Force (N)')
        plt.xlabel('Time (s)')
        plt.legend()

        plt.suptitle('LQR Controller Performance')
        plt.tight_layout()
        plt.savefig('lqr_performance.png')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down controller...')
        controller.save_and_plot_data()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
