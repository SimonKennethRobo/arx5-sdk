from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_arg = DeclareLaunchArgument("model", default_value="X5")
    interface_arg = DeclareLaunchArgument("interface", default_value="can0")
    control_mode_arg = DeclareLaunchArgument("control_mode", default_value="joint")
    publish_rate_arg = DeclareLaunchArgument("publish_rate", default_value="50.0")
    auto_home_arg = DeclareLaunchArgument("auto_home", default_value="false")
    gravity_compensation_arg = DeclareLaunchArgument("gravity_compensation", default_value="true")
    base_frame_arg = DeclareLaunchArgument("base_frame", default_value="base_link")
    joint_command_duration_arg = DeclareLaunchArgument("joint_command_duration", default_value="0.0")

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
            node,
        ]
    )
