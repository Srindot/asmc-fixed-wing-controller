#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
import math
from std_msgs.msg import Float64



class ASMCController:
    def __init__(self, lambda_gain, alpha0, alpha1, init_k0=1.0, init_k1=1.0):
        self.Lambda = lambda_gain
        self.alpha0 = alpha0
        self.alpha1 = alpha1
        self.K0_hat = init_k0
        self.K1_hat = init_k1
        self.prev_time = time.time()

    def compute(self, target, current):
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt <= 0.0:
            dt = 1e-5

        s = current - target  # Sliding variable

        # Adaptation laws
        self.K0_hat += (abs(s) - self.alpha0 * self.K0_hat) * dt
        self.K1_hat += (abs(s) * abs(current) - self.alpha1 * self.K1_hat) * dt

        rho = self.K0_hat + self.K1_hat * abs(current)

        # Control law
        control = -self.Lambda * s - rho * math.copysign(1.0, s)  # sgn(s)

        self.prev_time = current_time
        return control

    def set_publisher(self, node, joint_name):
        # Publishers for adaptive gains
        self.pub_k0 = node.create_publisher(Float64, f'/asmc_gains/{joint_name}/K0', 10)
        self.pub_k1 = node.create_publisher(Float64, f'/asmc_gains/{joint_name}/K1', 10)
        
        # Publishers for controller parameters
        self.pub_lambda = node.create_publisher(Float64, f'/asmc_params/{joint_name}/lambda', 10)
        self.pub_alpha0 = node.create_publisher(Float64, f'/asmc_params/{joint_name}/alpha0', 10)
        self.pub_alpha1 = node.create_publisher(Float64, f'/asmc_params/{joint_name}/alpha1', 10)

    def publish_gains(self):
        # Publish adaptive gains
        msg_k0 = Float64()
        msg_k0.data = self.K0_hat
        self.pub_k0.publish(msg_k0)

        msg_k1 = Float64()
        msg_k1.data = self.K1_hat
        self.pub_k1.publish(msg_k1)
        
        # Publish controller parameters
        msg_lambda = Float64()
        msg_lambda.data = self.Lambda
        self.pub_lambda.publish(msg_lambda)
        
        msg_alpha0 = Float64()
        msg_alpha0.data = self.alpha0
        self.pub_alpha0.publish(msg_alpha0)
        
        msg_alpha1 = Float64()
        msg_alpha1.data = self.alpha1
        self.pub_alpha1.publish(msg_alpha1)


class ASMCEffortPublisher(Node):
    def __init__(self):
        super().__init__('asmc_effort_publisher')

        self.joint_names = [
            'left_hip1_joint',
            'left_hip2_joint',
            'left_knee_joint',
            'right_hip1_joint',
            'right_hip2_joint',
            'right_knee_joint'
        ]

        for name in self.joint_names:
            self.declare_parameter(f'{name}.lambda')
            self.declare_parameter(f'{name}.alpha0')
            self.declare_parameter(f'{name}.alpha1')
            self.declare_parameter(f'{name}.target')
            
        self.controllers = {}
        self.target_positions = {}
        for name in self.joint_names:
            lam = self.get_parameter(f'{name}.lambda').get_parameter_value().double_value
            alpha0 = self.get_parameter(f'{name}.alpha0').get_parameter_value().double_value
            alpha1 = self.get_parameter(f'{name}.alpha1').get_parameter_value().double_value
            target = self.get_parameter(f'{name}.target').get_parameter_value().double_value
            self.controllers[name] = ASMCController(lam, alpha0, alpha1)
            self.target_positions[name] = target
            
            # Set up publishers for this joint
            self.controllers[name].set_publisher(self, name)

        self.current_positions = {name: 0.0 for name in self.joint_names}

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.target_sub = self.create_subscription(
            Float64MultiArray,
            '/target_positions',
            self.target_position_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )

        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        self.publish_timer = self.create_timer(0.5, self.publish_all_gains)  # 2 Hz for publishing gains

        self.get_logger().info('ASMC Effort Publisher Node started.')

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.current_positions:
                self.current_positions[name] = pos

    def target_position_callback(self, msg):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn("Received target array length mismatch.")
            return

        for i, name in enumerate(self.joint_names):
            self.target_positions[name] = msg.data[i]
        self.get_logger().info(f"Updated targets: {self.target_positions}")

    def publish_all_gains(self):
        for name in self.joint_names:
            self.controllers[name].publish_gains()

    def control_loop(self):
        efforts = []
        for name in self.joint_names:
            current = self.current_positions.get(name, 0.0)
            target = self.target_positions.get(name, 0.0)
            ctrl = self.controllers[name]
            effort = ctrl.compute(target, current)
            efforts.append(effort)

        msg = Float64MultiArray()
        msg.data = efforts
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ASMCEffortPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
