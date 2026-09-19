#include "arx5_ros2/arx5_ros2_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <csignal>
#include <sys/stat.h>

using namespace std::chrono_literals;

namespace
{

void quaternion_to_rpy(double x, double y, double z, double w, double &roll, double &pitch, double &yaw)
{
    roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    double sin_pitch = 2.0 * (w * y - z * x);
    pitch = std::abs(sin_pitch) >= 1.0 ? std::copysign(M_PI / 2.0, sin_pitch) : std::asin(sin_pitch);
    yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void rpy_to_quaternion(double roll, double pitch, double yaw, double &x, double &y, double &z, double &w)
{
    double cr = std::cos(roll / 2.0), sr = std::sin(roll / 2.0);
    double cp = std::cos(pitch / 2.0), sp = std::sin(pitch / 2.0);
    double cy = std::cos(yaw / 2.0), sy = std::sin(yaw / 2.0);
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
    w = cr * cp * cy + sr * sp * sy;
}

bool file_exists_nonempty(const std::string &path)
{
    struct stat info;
    return stat(path.c_str(), &info) == 0 && info.st_size > 0;
}

} // namespace

namespace arx5_ros2
{

Arx5Ros2Node::Arx5Ros2Node() : rclcpp::Node("arx5_controller")
{
    this->declare_parameter<std::string>("model", "X5");
    this->declare_parameter<std::string>("interface", "can0");
    this->declare_parameter<std::string>("control_mode", "joint");
    this->declare_parameter<double>("publish_rate", 50.0);
    this->declare_parameter<bool>("auto_home", false);
    this->declare_parameter<bool>("gravity_compensation", true);
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<double>("joint_command_duration", 0.0);
    // Gain scaling applied on top of the SDK's built-in default kp/kd for the
    // selected controller mode. Defaults match the values used by the known-working
    // hand-held phone teleop demo (ThuDemo), which softens the raw SDK defaults for
    // safe manual operation.
    this->declare_parameter<double>("kp_scale", 0.1);
    this->declare_parameter<double>("kd_scale", 0.5);
    // Gripper gain overrides; negative (default) keeps the SDK's built-in gripper gain.
    this->declare_parameter<double>("gripper_kp", 2.0);
    this->declare_parameter<double>("gripper_kd", -1.0);
    // Canonical Go2-X5 graph topics. They are parameters so the wrapper can
    // still be reused for a standalone arm without changing the SDK.
    this->declare_parameter<std::string>("state_topic", "/go2_x5/arm/state");
    this->declare_parameter<std::string>("command_topic", "/go2_x5/arm/command/target");
    this->declare_parameter<std::string>("mode_command_topic", "/go2_x5/arm/mode/target");
    this->declare_parameter<std::string>("mode_state_topic", "/go2_x5/arm/driver/mode");
    this->declare_parameter<std::string>("joint_name_prefix", "x5_joint");

    std::string model = this->get_parameter("model").as_string();
    std::string interface = this->get_parameter("interface").as_string();
    control_mode_ = this->get_parameter("control_mode").as_string();
    std::transform(control_mode_.begin(), control_mode_.end(), control_mode_.begin(), ::tolower);
    if (control_mode_ != "cartesian" && control_mode_ != "joint")
    {
        throw std::invalid_argument("control_mode must be 'cartesian' or 'joint'");
    }
    double publish_rate = this->get_parameter("publish_rate").as_double();
    if (publish_rate <= 0.0)
    {
        throw std::invalid_argument("publish_rate must be greater than zero");
    }
    joint_command_duration_ = this->get_parameter("joint_command_duration").as_double();
    if (joint_command_duration_ < 0.0)
    {
        throw std::invalid_argument("joint_command_duration must be greater than zero");
    }
    base_frame_ = this->get_parameter("base_frame").as_string();
    state_topic_ = this->get_parameter("state_topic").as_string();
    command_topic_ = this->get_parameter("command_topic").as_string();
    mode_command_topic_ = this->get_parameter("mode_command_topic").as_string();
    mode_state_topic_ = this->get_parameter("mode_state_topic").as_string();
    joint_name_prefix_ = this->get_parameter("joint_name_prefix").as_string();

    arx::RobotConfig robot_config = arx::RobotConfigFactory::get_instance().get_config(model);
    std::string urdf_path = std::string(ARX5_SDK_ROOT_DIR) + "/models/" + model + ".urdf";
    if (!file_exists_nonempty(urdf_path))
    {
        throw std::runtime_error("URDF file is missing or empty: " + urdf_path);
    }
    robot_config.urdf_path = urdf_path;
    joint_dof_ = robot_config.joint_dof;
    gripper_width_ = robot_config.gripper_width;

    arx::ControllerConfig controller_config = arx::ControllerConfigFactory::get_instance().get_config(
        control_mode_ + "_controller", robot_config.joint_dof);
    controller_config.gravity_compensation = this->get_parameter("gravity_compensation").as_bool();

    if (control_mode_ == "cartesian")
    {
        cartesian_controller_ =
            std::make_unique<arx::Arx5CartesianController>(robot_config, controller_config, interface);
        controller_ = cartesian_controller_.get();
    }
    else
    {
        joint_controller_ = std::make_unique<arx::Arx5JointController>(robot_config, controller_config, interface);
        controller_ = joint_controller_.get();
    }

    if (this->get_parameter("auto_home").as_bool())
    {
        RCLCPP_WARN(this->get_logger(), "Moving robot to home position");
        controller_->reset_to_home();
    }

    // The SDK starts in damping mode (kp=0). Enable the selected controller's
    // position gains while holding its current command.
    tracking_gain_ = std::make_unique<arx::Gain>(controller_config.default_kp, controller_config.default_kd,
                                                 controller_config.default_gripper_kp,
                                                 controller_config.default_gripper_kd);

    double kp_scale = this->get_parameter("kp_scale").as_double();
    double kd_scale = this->get_parameter("kd_scale").as_double();
    double gripper_kp = this->get_parameter("gripper_kp").as_double();
    double gripper_kd = this->get_parameter("gripper_kd").as_double();
    if (std::abs(kp_scale - 1.0) > 1e-6)
    {
        tracking_gain_->kp *= kp_scale;
    }
    if (std::abs(kd_scale - 1.0) > 1e-6)
    {
        tracking_gain_->kd *= kd_scale;
    }
    if (gripper_kp >= 0.0)
    {
        tracking_gain_->gripper_kp = static_cast<float>(gripper_kp);
    }
    if (gripper_kd >= 0.0)
    {
        tracking_gain_->gripper_kd = static_cast<float>(gripper_kd);
    }

    controller_->set_gain(*tracking_gain_);
    RCLCPP_INFO(this->get_logger(), "Position-control gains enabled (kp_scale=%.3f, kd_scale=%.3f, gripper_kp=%.3f, "
                                    "gripper_kd=%.3f)",
               kp_scale, kd_scale, tracking_gain_->gripper_kp, tracking_gain_->gripper_kd);

    arx::EEFState initial_eef = controller_->get_eef_state();
    arx::JointState initial_joint = controller_->get_joint_state();
    target_pose_ = initial_eef.pose_6d;
    target_joint_ = initial_joint.pos;
    target_gripper_ = initial_eef.gripper_pos;
    last_joint_command_log_time_ = this->now();

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(state_topic_, rclcpp::SensorDataQoS());
    mode_state_pub_ = this->create_publisher<std_msgs::msg::String>(mode_state_topic_, rclcpp::QoS(1).transient_local());
    eef_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/eef_state", rclcpp::SensorDataQoS());
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/gripper_state", rclcpp::SensorDataQoS());

    // The canonical graph always uses a JointTrajectory for arm positions.
    // Keep the old private Float64MultiArray endpoint for existing teleop
    // scripts while making the new endpoint independent of control_mode.
    joint_trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        command_topic_, rclcpp::QoS(10),
        std::bind(&Arx5Ros2Node::joint_trajectory_callback, this, std::placeholders::_1));
    mode_command_sub_ = this->create_subscription<std_msgs::msg::String>(
        mode_command_topic_, rclcpp::QoS(10),
        std::bind(&Arx5Ros2Node::mode_command_callback, this, std::placeholders::_1));

    if (control_mode_ == "cartesian")
    {
        eef_command_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/eef_cmd", 10, std::bind(&Arx5Ros2Node::eef_command_callback, this, std::placeholders::_1));
    }
    else
    {
        joint_command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "~/joint_cmd", 10, std::bind(&Arx5Ros2Node::joint_command_callback, this, std::placeholders::_1));
    }
    gripper_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "~/gripper_cmd", 10, std::bind(&Arx5Ros2Node::gripper_command_callback, this, std::placeholders::_1));

    reset_home_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reset_to_home", std::bind(&Arx5Ros2Node::reset_home_callback, this, std::placeholders::_1,
                                     std::placeholders::_2));
    float_mode_service_ = this->create_service<std_srvs::srv::SetBool>(
        "~/float_mode", std::bind(&Arx5Ros2Node::float_mode_callback, this, std::placeholders::_1,
                                  std::placeholders::_2));

    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate),
                                     std::bind(&Arx5Ros2Node::publish_state, this));

    RCLCPP_INFO(this->get_logger(), "ARX5 ready: model=%s, interface=%s, control_mode=%s, URDF=%s", model.c_str(),
               interface.c_str(), control_mode_.c_str(), urdf_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Canonical arm topics: state=%s command=%s mode=%s",
                state_topic_.c_str(), command_topic_.c_str(), mode_command_topic_.c_str());
    publish_mode_state();
}

void Arx5Ros2Node::send_target(double preview_time)
{
    if (floating_)
    {
        return;
    }
    // A zero timestamp tells the controller to fall back to its own
    // controller_config_.default_preview_time margin. Stamping an explicit
    // "now" (preview_time == 0) leaves no safety margin: by the time the
    // command reaches set_eef_cmd()/set_joint_cmd() (after IK, mutex, etc.)
    // real time may have already passed that timestamp, which throws
    // "End time must be no less than current time". Only stamp explicitly
    // when the caller asked for a non-zero preview/duration.
    if (control_mode_ == "cartesian")
    {
        arx::EEFState command;
        command.pose_6d = target_pose_;
        command.gripper_pos = target_gripper_;
        if (preview_time > 0.0)
        {
            command.timestamp = controller_->get_timestamp() + preview_time;
        }
        cartesian_controller_->set_eef_cmd(command);
    }
    else
    {
        arx::JointState command(joint_dof_);
        command.pos = target_joint_;
        command.gripper_pos = target_gripper_;
        if (preview_time > 0.0)
        {
            command.timestamp = controller_->get_timestamp() + preview_time;
        }
        joint_controller_->set_joint_cmd(command);
    }
}

void Arx5Ros2Node::send_target_after_float()
{
    // Set the measured state as the new target before restoring position gains.
    if (control_mode_ == "cartesian")
    {
        arx::EEFState command;
        command.pose_6d = target_pose_;
        command.gripper_pos = target_gripper_;
        command.timestamp = controller_->get_timestamp() + 0.1;
        cartesian_controller_->set_eef_cmd(command);
    }
    else
    {
        arx::JointState command(joint_dof_);
        command.pos = target_joint_;
        command.gripper_pos = target_gripper_;
        command.timestamp = controller_->get_timestamp() + 0.1;
        joint_controller_->set_joint_cmd(command);
    }
    controller_->set_gain(*tracking_gain_);
    floating_ = false;
}

void Arx5Ros2Node::eef_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (floating_)
    {
        RCLCPP_WARN(this->get_logger(), "Ignoring EEF command while in float mode");
        return;
    }
    const auto &orientation = msg->pose.orientation;
    double roll, pitch, yaw;
    quaternion_to_rpy(orientation.x, orientation.y, orientation.z, orientation.w, roll, pitch, yaw);
    target_pose_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, roll, pitch, yaw;
    send_target();
    RCLCPP_INFO(this->get_logger(), "Received EEF command: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", target_pose_[0],
               target_pose_[1], target_pose_[2], target_pose_[3], target_pose_[4], target_pose_[5]);
}

void Arx5Ros2Node::joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (floating_)
    {
        RCLCPP_WARN(this->get_logger(), "Ignoring joint command while in float mode");
        return;
    }
    if (static_cast<int>(msg->data.size()) != joint_dof_)
    {
        RCLCPP_ERROR(this->get_logger(), "Expected %d joint positions, received %zu", joint_dof_, msg->data.size());
        return;
    }
    for (int i = 0; i < joint_dof_; ++i)
    {
        target_joint_[i] = msg->data[i];
    }
    send_target(joint_command_duration_);
    rclcpp::Time now = this->now();
    if ((now - last_joint_command_log_time_).seconds() >= 1.0)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving continuous joint commands; duration=%.3fs",
                   joint_command_duration_);
        last_joint_command_log_time_ = now;
    }
}

void Arx5Ros2Node::joint_trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    if (floating_ || control_mode_ != "joint")
    {
        return;
    }
    if (msg->points.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Ignoring empty arm trajectory");
        return;
    }
    const auto &point = msg->points.back();
    if (static_cast<int>(point.positions.size()) != joint_dof_)
    {
        RCLCPP_ERROR(this->get_logger(), "Expected %d arm positions, received %zu", joint_dof_, point.positions.size());
        return;
    }
    if (!msg->joint_names.empty() && static_cast<int>(msg->joint_names.size()) == joint_dof_)
    {
        for (int i = 0; i < joint_dof_; ++i)
        {
            const std::string expected = joint_name_prefix_ + std::to_string(i + 1);
            if (msg->joint_names[i] != expected)
            {
                RCLCPP_ERROR(this->get_logger(), "Joint %d is '%s', expected '%s'", i,
                             msg->joint_names[i].c_str(), expected.c_str());
                return;
            }
        }
    }
    for (int i = 0; i < joint_dof_; ++i)
    {
        target_joint_[i] = point.positions[i];
    }
    double preview = static_cast<double>(point.time_from_start.sec) +
                     1e-9 * static_cast<double>(point.time_from_start.nanosec);
    if (preview <= 0.0) preview = joint_command_duration_;
    send_target(preview);
}

void Arx5Ros2Node::mode_command_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string mode = msg->data;
    std::transform(mode.begin(), mode.end(), mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    try
    {
        if (mode == "HOME")
        {
            controller_->reset_to_home();
            floating_ = false;
            controller_->set_gain(*tracking_gain_);
            current_mode_ = "HOME";
        }
        else if (mode == "HOLD")
        {
            const auto state = controller_->get_joint_state();
            target_joint_ = state.pos;
            const auto eef = controller_->get_eef_state();
            target_pose_ = eef.pose_6d;
            target_gripper_ = eef.gripper_pos;
            // Use the SDK joint-controller defaults directly; HOLD should
            // not inherit the ROS wrapper's softened gain scales.
            const auto config = controller_->get_controller_config();
            arx::Gain sdk_gain(config.default_kp, config.default_kd,
                               config.default_gripper_kp, config.default_gripper_kd);
            controller_->set_gain(sdk_gain);
            send_target();
            floating_ = false;
            current_mode_ = "HOLD";
        }
        else if (mode == "DAMPING")
        {
            // Native SDK damping: set_to_damping() installs the SDK default
            // derivative damping and a fixed zero-velocity command. Do not
            // enter the ROS wrapper's reduced-gain floating mode.
            controller_->set_to_damping();
            floating_ = false;
            current_mode_ = "DAMPING";
        }
        else if (mode == "OCS2")
        {
            if (floating_)
            {
                const auto state = controller_->get_joint_state();
                target_joint_ = state.pos;
                const auto eef = controller_->get_eef_state();
                target_pose_ = eef.pose_6d;
                target_gripper_ = eef.gripper_pos;
                send_target_after_float();
            }
            floating_ = false;
            current_mode_ = "OCS2";
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown arm mode '%s'", msg->data.c_str());
            return;
        }
        publish_mode_state();
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to enter arm mode %s: %s", mode.c_str(), error.what());
    }
}

void Arx5Ros2Node::publish_mode_state()
{
    if (!mode_state_pub_) return;
    std_msgs::msg::String msg;
    msg.data = current_mode_;
    mode_state_pub_->publish(msg);
}

void Arx5Ros2Node::gripper_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    if (floating_)
    {
        RCLCPP_WARN(this->get_logger(), "Ignoring gripper command while in float mode");
        return;
    }
    target_gripper_ = std::min(std::max(msg->data, 0.0), gripper_width_);
    send_target();
    RCLCPP_INFO(this->get_logger(), "Received gripper command: %.4f m", target_gripper_);
}

void Arx5Ros2Node::reset_home_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    try
    {
        controller_->reset_to_home();
        floating_ = false;
        controller_->set_gain(*tracking_gain_);
        arx::EEFState state = controller_->get_eef_state();
        arx::JointState joint_state = controller_->get_joint_state();
        target_pose_ = state.pose_6d;
        target_joint_ = joint_state.pos;
        target_gripper_ = state.gripper_pos;
        response->success = true;
        response->message = "Robot moved to home position";
    }
    catch (const std::exception &error)
    {
        response->success = false;
        response->message = error.what();
    }
}

void Arx5Ros2Node::float_mode_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                       std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    try
    {
        if (request->data && !floating_)
        {
            controller_->set_to_damping();
            arx::Gain floating_gain = controller_->get_gain();
            floating_gain.kd *= 0.1;
            controller_->set_gain(floating_gain);
            floating_ = true;
            response->message = "Float mode enabled";
        }
        else if (!request->data && floating_)
        {
            arx::EEFState state = controller_->get_eef_state();
            arx::JointState joint_state = controller_->get_joint_state();
            target_pose_ = state.pose_6d;
            target_joint_ = joint_state.pos;
            target_gripper_ = state.gripper_pos;
            send_target_after_float();
            response->message = "Float mode disabled; holding current pose";
        }
        else
        {
            response->message = "Float mode already in requested state";
        }
        response->success = true;
    }
    catch (const std::exception &error)
    {
        response->success = false;
        response->message = error.what();
    }
}

void Arx5Ros2Node::publish_state()
{
    arx::JointState joint_state = controller_->get_joint_state();
    arx::EEFState eef_state = controller_->get_eef_state();
    rclcpp::Time stamp = this->get_clock()->now();

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = stamp;
    joint_msg.name.resize(joint_dof_);
    joint_msg.position.resize(joint_dof_);
    joint_msg.velocity.resize(joint_dof_);
    joint_msg.effort.resize(joint_dof_);
    for (int i = 0; i < joint_dof_; ++i)
    {
        joint_msg.name[i] = joint_name_prefix_ + std::to_string(i + 1);
        joint_msg.position[i] = joint_state.pos[i];
        joint_msg.velocity[i] = joint_state.vel[i];
        joint_msg.effort[i] = joint_state.torque[i];
    }
    joint_pub_->publish(joint_msg);

    double qx, qy, qz, qw;
    rpy_to_quaternion(eef_state.pose_6d[3], eef_state.pose_6d[4], eef_state.pose_6d[5], qx, qy, qz, qw);
    geometry_msgs::msg::PoseStamped eef_msg;
    eef_msg.header.stamp = stamp;
    eef_msg.header.frame_id = base_frame_;
    eef_msg.pose.position.x = eef_state.pose_6d[0];
    eef_msg.pose.position.y = eef_state.pose_6d[1];
    eef_msg.pose.position.z = eef_state.pose_6d[2];
    eef_msg.pose.orientation.x = qx;
    eef_msg.pose.orientation.y = qy;
    eef_msg.pose.orientation.z = qz;
    eef_msg.pose.orientation.w = qw;
    eef_pub_->publish(eef_msg);

    std_msgs::msg::Float64 gripper_msg;
    gripper_msg.data = eef_state.gripper_pos;
    gripper_pub_->publish(gripper_msg);
}

void Arx5Ros2Node::stop()
{
    RCLCPP_WARN(this->get_logger(), "Node stopping: returning home, then entering damping");
    try
    {
        controller_->reset_to_home();
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to return home: %s", error.what());
    }
    controller_->set_to_damping();
    current_mode_ = "DAMPING";
}

} // namespace arx5_ros2

namespace
{
void request_shutdown(int /*signum*/)
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGTERM, request_shutdown);

    std::shared_ptr<arx5_ros2::Arx5Ros2Node> node;
    try
    {
        node = std::make_shared<arx5_ros2::Arx5Ros2Node>();
    }
    catch (const std::exception &error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("arx5_ros2"), "Failed to start ARX5 node: %s", error.what());
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return 1;
    }

    rclcpp::spin(node);

    node->stop();
    node.reset();
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return 0;
}
