from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument("model", default_value="X5")
    interface_arg = DeclareLaunchArgument("interface", default_value="can0")
    # control_mode_arg = DeclareLaunchArgument("control_mode", default_value="joint")
    control_mode_arg = DeclareLaunchArgument("control_mode", default_value="joint")
    publish_rate_arg = DeclareLaunchArgument("publish_rate", default_value="50.0")
    auto_home_arg = DeclareLaunchArgument("auto_home", default_value="false")
    gravity_compensation_arg = DeclareLaunchArgument("gravity_compensation", default_value="true")
    base_frame_arg = DeclareLaunchArgument("base_frame", default_value="base_link")
    joint_command_duration_arg = DeclareLaunchArgument("joint_command_duration", default_value="0.0")
    # Gain scaling on top of the SDK's default kp/kd. Defaults match the known-working
    # hand-held phone teleop demo (softened gains for safe manual operation).
    kp_scale_arg = DeclareLaunchArgument("kp_scale", default_value="0.1")
    kd_scale_arg = DeclareLaunchArgument("kd_scale", default_value="0.5")
    gripper_kp_arg = DeclareLaunchArgument("gripper_kp", default_value="2.0")
    gripper_kd_arg = DeclareLaunchArgument("gripper_kd", default_value="-1.0")
    state_topic_arg = DeclareLaunchArgument("state_topic", default_value="/go2_x5/arm/state")
    command_topic_arg = DeclareLaunchArgument("command_topic", default_value="/go2_x5/arm/command/target")
    mode_command_topic_arg = DeclareLaunchArgument("mode_command_topic", default_value="/go2_x5/arm/mode/target")
    mode_state_topic_arg = DeclareLaunchArgument("mode_state_topic", default_value="/go2_x5/arm/driver/mode")
    joint_name_prefix_arg = DeclareLaunchArgument("joint_name_prefix", default_value="x5_joint")

    node = Node(
        package="arx5_ros2",
        executable="arx5_ros2_node",
        name="arx5_controller",
        output="screen",
        parameters=[
            {
                "model": LaunchConfiguration("model"),
                "interface": LaunchConfiguration("interface"),
                "control_mode": LaunchConfiguration("control_mode"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "auto_home": LaunchConfiguration("auto_home"),
                "gravity_compensation": LaunchConfiguration("gravity_compensation"),
                "base_frame": LaunchConfiguration("base_frame"),
                "joint_command_duration": LaunchConfiguration("joint_command_duration"),
                "kp_scale": LaunchConfiguration("kp_scale"),
                "kd_scale": LaunchConfiguration("kd_scale"),
                "gripper_kp": LaunchConfiguration("gripper_kp"),
                "gripper_kd": LaunchConfiguration("gripper_kd"),
                "state_topic": LaunchConfiguration("state_topic"),
                "command_topic": LaunchConfiguration("command_topic"),
                "mode_command_topic": LaunchConfiguration("mode_command_topic"),
                "mode_state_topic": LaunchConfiguration("mode_state_topic"),
                "joint_name_prefix": LaunchConfiguration("joint_name_prefix"),
            }
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            interface_arg,
            control_mode_arg,
            publish_rate_arg,
            auto_home_arg,
            gravity_compensation_arg,
            base_frame_arg,
            joint_command_duration_arg,
            kp_scale_arg,
            kd_scale_arg,
            gripper_kp_arg,
            gripper_kd_arg,
            state_topic_arg,
            command_topic_arg,
            mode_command_topic_arg,
            mode_state_topic_arg,
            joint_name_prefix_arg,
            node,
        ]
    )
