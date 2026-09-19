#ifndef CONFIG_H
#define CONFIG_H

#include "app/common.h"
#include "utils.h"
#include <memory>
#include <unordered_map>
#include <vector>
#include <utility>
namespace arx
{

class RobotConfig
{
  public:
    std::string robot_model;
    std::string interface_name;

    VecDoF joint_pos_min;
    VecDoF joint_pos_max;
    VecDoF joint_vel_max;    // rad/s
    VecDoF joint_torque_max; // N*m
    Pose6d ee_vel_max;       // Currently not in used
    // end effector speed: m/s for (x, y, z), rad/s for (roll, pitch, yaw)

    double gripper_vel_max; // m/s
    double gripper_torque_max;
    double gripper_width;        // m, fully opened: gripper_width, fully closed: 0
    double gripper_open_readout; // fully-opened gripper motor readout. Should be calibrated using
                                 // python/examples/calibrate.py
    int joint_dof;
    std::vector<int> motor_id;
    std::vector<MotorType> motor_type;
    int gripper_motor_id;
    MotorType gripper_motor_type; // Set to MotorType::NONE if the robot does not have a gripper

    // Will be used in inverse dynamics calculation.
    // Please change it to other values if the robot arm is not placed on the ground.
    Eigen::Vector3d gravity_vector;

    // Will be used in IK and FK.
    // ID will find stop at last active joint (instead of the eef link with a fixed joint) because of some KDL bugs
    std::string base_link_name;
    std::string eef_link_name;

    std::string urdf_path;

    RobotConfig(std::string robot_model, VecDoF joint_pos_min, VecDoF joint_pos_max, VecDoF joint_vel_max,
                VecDoF joint_torque_max, Pose6d ee_vel_max, double gripper_vel_max, double gripper_torque_max,
                double gripper_width, double gripper_open_readout, int joint_dof, std::vector<int> motor_id,
                std::vector<MotorType> motor_type, int gripper_motor_id, MotorType gripper_motor_type,
                Eigen::Vector3d gravity_vector, std::string base_link_name, std::string eef_link_name,
                std::string urdf_path)
        : robot_model(robot_model), joint_pos_min(joint_pos_min), joint_pos_max(joint_pos_max),
          joint_vel_max(joint_vel_max), joint_torque_max(joint_torque_max), ee_vel_max(ee_vel_max),
          gripper_vel_max(gripper_vel_max), gripper_torque_max(gripper_torque_max), gripper_width(gripper_width),
          gripper_open_readout(gripper_open_readout), joint_dof(joint_dof), motor_id(motor_id), motor_type(motor_type),
          gripper_motor_id(gripper_motor_id), gripper_motor_type(gripper_motor_type), gravity_vector(gravity_vector),
          base_link_name(base_link_name), eef_link_name(eef_link_name), urdf_path(urdf_path)
    {
    }
};

class RobotConfigFactory
{
  public:
    static RobotConfigFactory &get_instance();
    RobotConfig get_config(const std::string &robot_model) const;
};

class ControllerConfig
{
  public:
    std::string controller_type;
    VecDoF default_kp;
    VecDoF default_kd;
    double default_gripper_kp;
    double default_gripper_kd;
    int over_current_cnt_max;
    double controller_dt;
    bool gravity_compensation;
    bool background_send_recv;
    bool shutdown_to_passive;
    // true: will set the arm to damping then passive mode when pressing `ctrl-C`. (recommended);
    //       pressing `ctrl-\` will directly kill the program so this process will be skipped
    // false: will keep the arm in the air when shutting down the controller (both `ctrl-\` and `ctrl-C`).
    //       X5 cannot be kept in the air.
    std::string interpolation_method; // "linear" or "cubic" (cubic is not well supported yet)
    double default_preview_time;      // The default value for preview time if the command has 0 timestamp

    ControllerConfig(std::string controller_type, VecDoF default_kp, VecDoF default_kd, double default_gripper_kp,
                     double default_gripper_kd, int over_current_cnt_max, double controller_dt,
                     bool gravity_compensation, bool background_send_recv, bool shutdown_to_passive,
                     std::string interpolation_method, double default_preview_time)
        : controller_type(controller_type), default_kp(default_kp), default_kd(default_kd),
          default_gripper_kp(default_gripper_kp), default_gripper_kd(default_gripper_kd),
          over_current_cnt_max(over_current_cnt_max), controller_dt(controller_dt),
          gravity_compensation(gravity_compensation), background_send_recv(background_send_recv),
          shutdown_to_passive(shutdown_to_passive), interpolation_method(interpolation_method),
          default_preview_time(default_preview_time)
    {
    }
};

class ControllerConfigFactory
{
  public:
    static ControllerConfigFactory &get_instance();
    ControllerConfig get_config(const std::string &controller_type, int joint_dof) const;
};

// Runtime YAML configuration. The explicit path wins, followed by ARX5_CONFIG_FILE
// and <sdk-root>/config/arx5.yaml. Existing factory APIs remain compatible.
RobotConfig load_robot_config(const std::string &robot_model, const std::string &config_file = "");
ControllerConfig load_controller_config(const std::string &controller_type, int joint_dof,
                                        const std::string &config_file = "");
std::string resolve_config_file(const std::string &config_file = "");

} // namespace arx

#endif // CONFIG_H