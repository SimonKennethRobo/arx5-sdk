#!/usr/bin/env python3
"""Publish a predefined list of ARX5 joint-position commands."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Absolute joint positions in radians, ordered as joint1 ... joint6.
# Edit this list to define the test sequence.
JOINT_POSITION_COMMANDS = [
    [0.0, 0.2, 0.2, 0.0, 0.0, 0.0],
    [0.1, 0.3, 0.3, 0.0, 0.0, 0.0],
    [-0.1, 0.3, 0.3, 0.0, 0.0, 0.0],
    [0.0, 0.2, 0.2, 0.0, 0.0, 0.0],
]


class JointCommandListPublisher(Node):
    def __init__(self):
        super().__init__("arx5_joint_command_test")
        self.declare_parameter("command_topic", "/arx5_controller/joint_command")
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("command_duration", 2.0)
        self.declare_parameter("return_to_start", True)

        command_topic = self.get_parameter("command_topic").value
        state_topic = self.get_parameter("state_topic").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.command_duration = float(self.get_parameter("command_duration").value)
        self.return_to_start = bool(self.get_parameter("return_to_start").value)
        if self.publish_rate <= 0.0 or self.command_duration <= 0.0:
            raise ValueError(
                "publish_rate and command_duration must be greater than zero"
            )
        if not JOINT_POSITION_COMMANDS:
            raise ValueError("JOINT_POSITION_COMMANDS must not be empty")

        self.publisher = self.create_publisher(Float64MultiArray, command_topic, 10)
        self.subscription = self.create_subscription(
            JointState, state_topic, self.state_callback, qos_profile_sensor_data
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.start_positions = None
        self.command_index = 0
        self.command_start_time = None
        self.finished = False
        self.get_logger().info(
            f"Waiting for initial state on {state_topic}; publishing to {command_topic}"
        )

    def state_callback(self, message):
        if self.start_positions is not None or not message.position:
            return

        joint_count = len(message.position)
        invalid = [
            index
            for index, command in enumerate(JOINT_POSITION_COMMANDS)
            if len(command) != joint_count
        ]
        if invalid:
            self.get_logger().error(
                f"Commands at indexes {invalid} do not contain {joint_count} positions"
            )
            self.finished = True
            return

        self.start_positions = list(message.position)
        self.command_start_time = time.monotonic()
        self.get_logger().warning(
            f"Starting sequence of {len(JOINT_POSITION_COMMANDS)} joint commands"
        )
        self.log_current_command()

    def timer_callback(self):
        if self.finished or self.start_positions is None:
            return

        elapsed = time.monotonic() - self.command_start_time
        if elapsed >= self.command_duration:
            self.command_index += 1
            self.command_start_time = time.monotonic()
            if self.command_index >= len(JOINT_POSITION_COMMANDS):
                if self.return_to_start:
                    self.publisher.publish(Float64MultiArray(data=self.start_positions))
                    self.get_logger().info(
                        "Sequence finished; commanded the initial joint positions"
                    )
                else:
                    self.get_logger().info("Joint command sequence finished")
                self.finished = True
                return
            self.log_current_command()

        command = JOINT_POSITION_COMMANDS[self.command_index]
        self.publisher.publish(Float64MultiArray(data=command))

    def log_current_command(self):
        command = JOINT_POSITION_COMMANDS[self.command_index]
        self.get_logger().info(
            f"Command {self.command_index + 1}/{len(JOINT_POSITION_COMMANDS)}: "
            f"{command} rad"
        )

    def publish_start_positions(self):
        if self.start_positions is not None and self.return_to_start:
            self.publisher.publish(Float64MultiArray(data=self.start_positions))


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandListPublisher()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.publish_start_positions()
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
