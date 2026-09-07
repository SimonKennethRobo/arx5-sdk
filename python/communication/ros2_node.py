#!/usr/bin/env python3
"""ROS 2 Foxy wrapper for the ARX5 Cartesian controller."""

import math
import os
import signal
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState as JointStateMsg
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

COMMUNICATION_DIR = os.path.dirname(os.path.abspath(__file__))
PYTHON_DIR = os.path.dirname(COMMUNICATION_DIR)
SDK_ROOT_DIR = os.path.dirname(PYTHON_DIR)
sys.path.insert(0, PYTHON_DIR)
import arx5_interface as arx5  # noqa: E402


def quaternion_to_rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1 else math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rpy_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Arx5Ros2Node(Node):
    def __init__(self):
        super().__init__("arx5_controller")
        self.declare_parameter("model", "X5")
        self.declare_parameter("interface", "can0")
        self.declare_parameter("control_mode", "cartesian")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("auto_home", False)
        self.declare_parameter("gravity_compensation", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("joint_command_duration", 0)

        model = self.get_parameter("model").value
        interface = self.get_parameter("interface").value
        self.control_mode = str(self.get_parameter("control_mode").value).lower()
        if self.control_mode not in ("cartesian", "joint"):
            raise ValueError("control_mode must be 'cartesian' or 'joint'")
        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        self.joint_command_duration = float(self.get_parameter("joint_command_duration").value)
        if self.joint_command_duration < 0:
            raise ValueError("joint_command_duration must be greater than zero")

        robot_config = arx5.RobotConfigFactory.get_instance().get_config(model)
        urdf_path = os.path.join(SDK_ROOT_DIR, "models", f"{model}.urdf")
        if not os.path.isfile(urdf_path) or os.path.getsize(urdf_path) == 0:
            raise FileNotFoundError(f"URDF file is missing or empty: {urdf_path}")
        robot_config.urdf_path = urdf_path

        controller_config = arx5.ControllerConfigFactory.get_instance().get_config(
            f"{self.control_mode}_controller", robot_config.joint_dof
        )
        controller_config.gravity_compensation = bool(self.get_parameter("gravity_compensation").value)
        controller_class = (
            arx5.Arx5CartesianController if self.control_mode == "cartesian" else arx5.Arx5JointController
        )
        self.controller = controller_class(robot_config, controller_config, interface)
        self.robot_config = robot_config
        self.base_frame = self.get_parameter("base_frame").value
        self.floating = False
        self.last_joint_command_log_time = 0.0

        if self.get_parameter("auto_home").value:
            self.get_logger().warning("Moving robot to home position")
            self.controller.reset_to_home()

        # The SDK starts in damping mode (kp=0). Enable the selected
        # controller's position gains while holding its current command.
        self.tracking_gain = arx5.Gain(
            controller_config.default_kp.copy(),
            controller_config.default_kd.copy(),
            controller_config.default_gripper_kp,
            controller_config.default_gripper_kd,
        )
        self.controller.set_gain(self.tracking_gain)
        self.get_logger().info("Position-control gains enabled")
        initial_eef = self.controller.get_eef_state()
        initial_joint = self.controller.get_joint_state()
        self.target_pose = initial_eef.pose_6d().copy()
        self.target_joint = initial_joint.pos().copy()
        self.target_gripper = float(initial_eef.gripper_pos)

        self.joint_pub = self.create_publisher(JointStateMsg, "/joint_states", qos_profile_sensor_data)
        self.eef_pub = self.create_publisher(PoseStamped, "~/eef_state", qos_profile_sensor_data)
        self.gripper_pub = self.create_publisher(Float64, "~/gripper_state", qos_profile_sensor_data)
        if self.control_mode == "cartesian":
            self.create_subscription(PoseStamped, "~/eef_command", self.eef_command_callback, 10)
        else:
            self.create_subscription(
                Float64MultiArray,
                "~/joint_command",
                self.joint_command_callback,
                10,
            )
        self.create_subscription(Float64, "~/gripper_command", self.gripper_command_callback, 10)
        self.create_service(Trigger, "~/reset_to_home", self.reset_home_callback)
        self.create_service(SetBool, "~/float_mode", self.float_mode_callback)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_state)

        self.get_logger().info(
            f"ARX5 ready: model={model}, interface={interface}, control_mode={self.control_mode}, URDF={urdf_path}"
        )
        command_topic = "~/eef_command" if self.control_mode == "cartesian" else "~/joint_command"
        self.get_logger().info(f"Listening for commands on {command_topic}")

    def send_target(self, preview_time=0):
        if self.floating:
            return
        if self.control_mode == "cartesian":
            command = arx5.EEFState()
            command.pose_6d()[:] = self.target_pose
            command.gripper_pos = self.target_gripper
            command.timestamp = self.controller.get_timestamp() + preview_time
            self.controller.set_eef_cmd(command)
        else:
            command = arx5.JointState(self.robot_config.joint_dof)
            command.pos()[:] = self.target_joint
            command.gripper_pos = self.target_gripper
            command.timestamp = self.controller.get_timestamp() + preview_time
            self.controller.set_joint_cmd(command)

    def eef_command_callback(self, message):
        if self.floating:
            self.get_logger().warning("Ignoring EEF command while in float mode")
            return
        pose = message.pose
        roll, pitch, yaw = quaternion_to_rpy(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self.target_pose[:] = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            roll,
            pitch,
            yaw,
        )
        self.send_target()
        self.get_logger().info(f"Received EEF command: [{', '.join(f'{value:.3f}' for value in self.target_pose)}]")

    def joint_command_callback(self, message):
        if self.floating:
            self.get_logger().warning("Ignoring joint command while in float mode")
            return
        positions = message.data
        if len(positions) != self.robot_config.joint_dof:
            self.get_logger().error(
                f"Expected {self.robot_config.joint_dof} joint positions, received {len(positions)}"
            )
            return

        self.target_joint[:] = positions
        self.send_target(self.joint_command_duration)
        now = time.monotonic()
        if now - self.last_joint_command_log_time >= 1.0:
            self.get_logger().info(
                "Receiving continuous joint commands; latest: "
                f"[{', '.join(f'{value:.3f}' for value in self.target_joint)}], "
                f"duration={self.joint_command_duration:.3f}s"
            )
            self.last_joint_command_log_time = now

    def gripper_command_callback(self, message):
        if self.floating:
            self.get_logger().warning("Ignoring gripper command while in float mode")
            return
        self.target_gripper = float(np.clip(message.data, 0.0, self.robot_config.gripper_width))
        self.send_target()
        self.get_logger().info(f"Received gripper command: {self.target_gripper:.4f} m")

    def reset_home_callback(self, request, response):
        del request
        try:
            self.controller.reset_to_home()
            self.floating = False
            self.controller.set_gain(self.tracking_gain)
            state = self.controller.get_eef_state()
            joint_state = self.controller.get_joint_state()
            self.target_pose = state.pose_6d().copy()
            self.target_joint = joint_state.pos().copy()
            self.target_gripper = float(state.gripper_pos)
            response.success = True
            response.message = "Robot moved to home position"
        except Exception as error:  # ROS service must return the failure to its caller.
            response.success = False
            response.message = str(error)
        return response

    def float_mode_callback(self, request, response):
        try:
            if request.data and not self.floating:
                self.controller.set_to_damping()
                floating_gain = self.controller.get_gain()
                floating_gain.kd()[:] *= 0.1
                self.controller.set_gain(floating_gain)
                self.floating = True
                response.message = "Float mode enabled"
            elif not request.data and self.floating:
                state = self.controller.get_eef_state()
                joint_state = self.controller.get_joint_state()
                self.target_pose = state.pose_6d().copy()
                self.target_joint = joint_state.pos().copy()
                self.target_gripper = float(state.gripper_pos)
                self.send_target_after_float()
                response.message = "Float mode disabled; holding current pose"
            else:
                response.message = "Float mode already in requested state"
            response.success = True
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def send_target_after_float(self):
        # Set the measured state as the new target before restoring position gains.
        if self.control_mode == "cartesian":
            command = arx5.EEFState()
            command.pose_6d()[:] = self.target_pose
            command.gripper_pos = self.target_gripper
            command.timestamp = self.controller.get_timestamp() + 0.1
            self.controller.set_eef_cmd(command)
        else:
            command = arx5.JointState(self.robot_config.joint_dof)
            command.pos()[:] = self.target_joint
            command.gripper_pos = self.target_gripper
            command.timestamp = self.controller.get_timestamp() + 0.1
            self.controller.set_joint_cmd(command)
        self.controller.set_gain(self.tracking_gain)
        self.floating = False

    def publish_state(self):
        joint_state = self.controller.get_joint_state()
        eef_state = self.controller.get_eef_state()
        stamp = self.get_clock().now().to_msg()

        joint_message = JointStateMsg()
        joint_message.header.stamp = stamp
        joint_message.name = [f"joint{index + 1}" for index in range(self.robot_config.joint_dof)]
        joint_message.position = joint_state.pos().tolist()
        joint_message.velocity = joint_state.vel().tolist()
        joint_message.effort = joint_state.torque().tolist()
        self.joint_pub.publish(joint_message)

        pose_6d = eef_state.pose_6d()
        quaternion = rpy_to_quaternion(*pose_6d[3:])
        eef_message = PoseStamped()
        eef_message.header.stamp = stamp
        eef_message.header.frame_id = self.base_frame
        eef_message.pose.position.x = float(pose_6d[0])
        eef_message.pose.position.y = float(pose_6d[1])
        eef_message.pose.position.z = float(pose_6d[2])
        eef_message.pose.orientation.x = quaternion[0]
        eef_message.pose.orientation.y = quaternion[1]
        eef_message.pose.orientation.z = quaternion[2]
        eef_message.pose.orientation.w = quaternion[3]
        self.eef_pub.publish(eef_message)
        self.gripper_pub.publish(Float64(data=float(eef_state.gripper_pos)))

    def stop(self):
        self.get_logger().warning("Node stopping: returning home, then entering damping")
        try:
            self.controller.reset_to_home()
        except Exception as error:
            self.get_logger().error(f"Failed to return home: {error}")
        finally:
            self.controller.set_to_damping()


def main(args=None):
    rclpy.init(args=args)
    node = None

    def request_shutdown(signum, frame):
        del signum, frame
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        node = Arx5Ros2Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGTERM shuts down the ROS context, which may make spin() raise on
        # some Foxy patch releases. Unexpected errors must still propagate.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
