#include "app/solver.h"

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <kdl/frames.hpp>

int main()
{
    constexpr int joint_dof = 6;
    Eigen::VectorXd joint_pos_min(joint_dof);
    Eigen::VectorXd joint_pos_max(joint_dof);
    joint_pos_min << -3.14, -0.05, -0.1, -1.6, -1.57, -2.0;
    joint_pos_max << 2.618, 3.50, 3.20, 1.55, 1.57, 2.0;

    arx::Arx5Solver solver(std::string(ARX5_SOURCE_DIR) + "/models/X5.urdf", joint_dof, joint_pos_min, joint_pos_max);
    Eigen::VectorXd expected_joint_pos(joint_dof);
    expected_joint_pos << 0.25, 0.8, 0.35, -0.4, 0.3, -0.2;
    const Eigen::Matrix<double, 6, 1> target_pose = solver.forward_kinematics(expected_joint_pos);

    const auto result = solver.dls_inverse_kinematics(target_pose, Eigen::VectorXd::Zero(joint_dof));
    const int status = std::get<0>(result);
    const Eigen::Matrix<double, 6, 1> solved_pose = solver.forward_kinematics(std::get<1>(result));

    const double position_error = (target_pose.head<3>() - solved_pose.head<3>()).norm();
    const KDL::Rotation target_rotation = KDL::Rotation::RPY(target_pose[3], target_pose[4], target_pose[5]);
    const KDL::Rotation solved_rotation = KDL::Rotation::RPY(solved_pose[3], solved_pose[4], solved_pose[5]);
    const double orientation_error = KDL::diff(solved_rotation, target_rotation).Norm();

    std::cout << "status=" << status << ", position_error=" << position_error
              << ", orientation_error=" << orientation_error << '\n';
    if (status != KDL::SolverI::E_NOERROR || position_error > 1E-4 || orientation_error > 1E-3)
    {
        return 1;
    }
    return 0;
}
