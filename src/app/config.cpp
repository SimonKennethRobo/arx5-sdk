#include "app/config.h"

#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace arx
{
namespace
{
std::string dirname(const std::string &path)
{
    const std::size_t slash = path.find_last_of("/");
    return slash == std::string::npos ? std::string(".") : path.substr(0, slash);
}

void require_node(const YAML::Node &node, const std::string &name)
{
    if (!node || node.IsNull())
        throw std::runtime_error("Missing YAML field: " + name);
}

VecDoF vector_node(const YAML::Node &node, const std::string &name, int expected)
{
    require_node(node, name);
    if (!node.IsSequence() || static_cast<int>(node.size()) != expected)
        throw std::runtime_error("YAML field " + name + " must contain exactly " + std::to_string(expected) + " values");
    VecDoF result(expected);
    for (int i = 0; i < expected; ++i)
        result[i] = node[i].as<double>();
    return result;
}

Pose6d pose_node(const YAML::Node &node, const std::string &name)
{
    VecDoF v = vector_node(node, name, 6);
    Pose6d result;
    result = v;
    return result;
}

Eigen::Vector3d gravity_node(const YAML::Node &node, const std::string &name)
{
    require_node(node, name);
    if (!node.IsSequence() || node.size() != 3)
        throw std::runtime_error("YAML field " + name + " must contain exactly 3 values");
    return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

MotorType motor_type(const YAML::Node &node, const std::string &name)
{
    const std::string value = node.as<std::string>();
    if (value == "EC_A4310") return MotorType::EC_A4310;
    if (value == "DM_J4310") return MotorType::DM_J4310;
    if (value == "DM_J4340") return MotorType::DM_J4340;
    if (value == "DM_J8009") return MotorType::DM_J8009;
    if (value == "NONE") return MotorType::NONE;
    throw std::runtime_error("Unknown motor type at " + name + ": " + value);
}

std::vector<MotorType> motor_types(const YAML::Node &node, int expected, const std::string &name)
{
    require_node(node, name);
    if (!node.IsSequence() || static_cast<int>(node.size()) != expected)
        throw std::runtime_error("YAML field " + name + " must contain exactly " + std::to_string(expected) + " values");
    std::vector<MotorType> result;
    for (int i = 0; i < expected; ++i)
        result.push_back(motor_type(node[i], name + "[" + std::to_string(i) + "]"));
    return result;
}

YAML::Node load_root(const std::string &path)
{
    try
    {
        YAML::Node root = YAML::LoadFile(path);
        if (root["version"] && root["version"].as<int>() != 1)
            throw std::runtime_error("Unsupported configuration version in " + path);
        return root;
    }
    catch (const YAML::Exception &e)
    {
        throw std::runtime_error("Failed to load YAML configuration '" + path + "': " + e.what());
    }
}
} // namespace

std::string resolve_config_file(const std::string &config_file)
{
    if (!config_file.empty()) return config_file;
    const char *env = std::getenv("ARX5_CONFIG_FILE");
    if (env && *env) return std::string(env);
#ifdef ARX5_SDK_ROOT_DIR
    const std::string compiled_root = std::string(ARX5_SDK_ROOT_DIR) + "/config/arx5.yaml";
    std::ifstream compiled_file(compiled_root.c_str());
    if (compiled_file.good()) return compiled_root;
#endif
    const std::string root = get_root_dir();
    const std::string local = root + "/config/arx5.yaml";
    std::ifstream local_file(local.c_str());
    if (local_file.good()) return local;
    const std::string installed = root + "/../share/arx5-sdk/config/arx5.yaml";
    std::ifstream installed_file(installed.c_str());
    if (installed_file.good()) return installed;
    const std::string ros_installed = root + "/../share/arx5_ros2/config/arx5.yaml";
    std::ifstream ros_installed_file(ros_installed.c_str());
    if (ros_installed_file.good()) return ros_installed;
    return local;
}

RobotConfig load_robot_config(const std::string &robot_model, const std::string &config_file)
{
    const std::string path = resolve_config_file(config_file);
    YAML::Node root = load_root(path);
    YAML::Node n = root["robots"][robot_model];
    if (!n || n.IsNull()) throw std::runtime_error("Unknown robot model in " + path + ": " + robot_model);
    const std::string prefix = "robots." + robot_model + ".";
    const int dof = n["joint_dof"].as<int>();
    if (dof <= 0) throw std::runtime_error(prefix + "joint_dof must be positive");
    YAML::Node gripper = n["gripper"];
    require_node(gripper, prefix + "gripper");
    require_node(n["motor_id"], prefix + "motor_id");
    require_node(n["motor_type"], prefix + "motor_type");
    const std::string urdf = n["urdf_path"].as<std::string>();
    const std::string urdf_path = (!urdf.empty() && urdf[0] == '/') ? urdf : dirname(path) + "/" + urdf;
    RobotConfig result(
        robot_model,
        vector_node(n["joint_pos_min"], prefix + "joint_pos_min", dof),
        vector_node(n["joint_pos_max"], prefix + "joint_pos_max", dof),
        vector_node(n["joint_vel_max"], prefix + "joint_vel_max", dof),
        vector_node(n["joint_torque_max"], prefix + "joint_torque_max", dof),
        pose_node(n["ee_vel_max"], prefix + "ee_vel_max"),
        gripper["vel_max"].as<double>(), gripper["torque_max"].as<double>(), gripper["width"].as<double>(),
        gripper["open_readout"].as<double>(), dof,
        [&]() { std::vector<int> ids; for (int i = 0; i < dof; ++i) { int id = n["motor_id"][i].as<int>(); if (id < 0 || id >= 10) throw std::runtime_error(prefix + "motor_id must be in [0, 9]"); ids.push_back(id); } return ids; }(),
        motor_types(n["motor_type"], dof, prefix + "motor_type"),
        gripper["motor_id"].as<int>(), motor_type(gripper["motor_type"], prefix + "gripper.motor_type"),
        gravity_node(n["gravity_vector"], prefix + "gravity_vector"), n["base_link_name"].as<std::string>(),
        n["eef_link_name"].as<std::string>(), urdf_path);
    if (result.gripper_motor_id < 0 || result.gripper_motor_id >= 10)
        throw std::runtime_error(prefix + "gripper.motor_id must be in [0, 9]");
    for (int i = 0; i < dof; ++i)
        if (result.joint_pos_min[i] > result.joint_pos_max[i])
            throw std::runtime_error(prefix + "joint_pos_min is greater than joint_pos_max");
    return result;
}

ControllerConfig load_controller_config(const std::string &controller_type, int joint_dof,
                                        const std::string &config_file)
{
    const std::string path = resolve_config_file(config_file);
    YAML::Node root = load_root(path);
    YAML::Node n = root["controllers"][controller_type][std::to_string(joint_dof)];
    if (!n || n.IsNull())
        throw std::runtime_error("Unknown controller configuration in " + path + ": " + controller_type + "/" + std::to_string(joint_dof));
    const std::string prefix = "controllers." + controller_type + "." + std::to_string(joint_dof) + ".";
    return ControllerConfig(
        controller_type, vector_node(n["default_kp"], prefix + "default_kp", joint_dof),
        vector_node(n["default_kd"], prefix + "default_kd", joint_dof), n["default_gripper_kp"].as<double>(),
        n["default_gripper_kd"].as<double>(), n["over_current_cnt_max"].as<int>(), n["controller_dt"].as<double>(),
        n["gravity_compensation"].as<bool>(), n["background_send_recv"].as<bool>(),
        n["shutdown_to_passive"].as<bool>(), n["interpolation_method"].as<std::string>(),
        n["default_preview_time"].as<double>());
}

RobotConfigFactory &RobotConfigFactory::get_instance()
{
    static RobotConfigFactory instance;
    return instance;
}

RobotConfig RobotConfigFactory::get_config(const std::string &robot_model) const
{
    return load_robot_config(robot_model);
}

ControllerConfigFactory &ControllerConfigFactory::get_instance()
{
    static ControllerConfigFactory instance;
    return instance;
}

ControllerConfig ControllerConfigFactory::get_config(const std::string &controller_type, int joint_dof) const
{
    return load_controller_config(controller_type, joint_dof);
}

} // namespace arx
