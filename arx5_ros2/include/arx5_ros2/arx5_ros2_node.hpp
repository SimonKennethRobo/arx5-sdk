#ifndef ARX5_ROS2_NODE_HPP
#define ARX5_ROS2_NODE_HPP

#include "app/cartesian_controller.h"
#include "app/common.h"
#include "app/config.h"
#include "app/joint_controller.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

namespace arx5_ros2
{

class Arx5Ros2Node : public rclcpp::Node
{
  public:
    Arx5Ros2Node();

    // Returns the robot to its home position and enters damping mode.
    // Must be called (from main, after spin() returns) before the node is destroyed.
    void stop();

  private:
    void send_target(double preview_time = 0.0);
    void send_target_after_float();

    void eef_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void joint_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void mode_command_callback(const std_msgs::msg::String::SharedPtr msg);
    void gripper_command_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void reset_home_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void float_mode_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                             std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void publish_state();
    void publish_mode_state();

    // Active controller (owned by exactly one of the two unique_ptrs below).
    std::unique_ptr<arx::Arx5CartesianController> cartesian_controller_;
    std::unique_ptr<arx::Arx5JointController> joint_controller_;
    arx::Arx5ControllerBase *controller_ = nullptr;

    std::string control_mode_;
    std::string base_frame_;
    double joint_command_duration_ = 0.0;
    int joint_dof_ = 0;
    double gripper_width_ = 0.0;
    bool floating_ = false;
    std::string state_topic_;
    std::string command_topic_;
    std::string mode_command_topic_;
    std::string mode_state_topic_;
    std::string joint_name_prefix_;
    std::string current_mode_ = "HOLD";

    std::unique_ptr<arx::Gain> tracking_gain_;
    arx::Pose6d target_pose_ = arx::Pose6d::Zero();
    arx::VecDoF target_joint_;
    double target_gripper_ = 0.0;

    rclcpp::Time last_joint_command_log_time_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr eef_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr eef_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_command_sub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_home_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr float_mode_service_;

    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace arx5_ros2

#endif // ARX5_ROS2_NODE_HPP
