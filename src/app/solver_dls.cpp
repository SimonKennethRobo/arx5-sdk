#include "app/solver.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <kdl/chainjnttojacsolver.hpp>
#include <limits>
#include <stdexcept>

namespace arx
{

std::tuple<int, Eigen::VectorXd>
Arx5Solver::dls_inverse_kinematics(Eigen::Matrix<double, 6, 1> target_pose_6d, Eigen::VectorXd current_joint_pos,
                                   double damping, int max_iterations, double position_tolerance,
                                   double orientation_tolerance, double max_joint_step, double position_weight,
                                   double orientation_weight)
{
    if (current_joint_pos.size() != JOINT_DOF_)
    {
        throw std::invalid_argument("current_joint_pos size must equal the robot joint DOF");
    }
    if (JOINT_POS_MIN_.size() != JOINT_DOF_ || JOINT_POS_MAX_.size() != JOINT_DOF_)
    {
        throw std::runtime_error("joint limit size does not equal the robot joint DOF");
    }
    if (!target_pose_6d.allFinite() || !current_joint_pos.allFinite())
    {
        throw std::invalid_argument("target pose and current joint position must contain only finite values");
    }
    if (damping <= 0.0 || max_iterations <= 0 || position_tolerance <= 0.0 ||
        orientation_tolerance <= 0.0 || max_joint_step <= 0.0)
    {
        throw std::invalid_argument("DLS parameters must be positive");
    }
    if (position_weight <= 0.0 || orientation_weight <= 0.0)
    {
        throw std::invalid_argument("DLS error weights must be positive");
    }

    Eigen::Matrix<double, 6, 1> error_weight;
    error_weight << position_weight, position_weight, position_weight, orientation_weight, orientation_weight,
        orientation_weight;

    Eigen::VectorXd joint_pos = current_joint_pos.cwiseMax(JOINT_POS_MIN_).cwiseMin(JOINT_POS_MAX_);
    Eigen::VectorXd best_joint_pos = joint_pos;
    double best_normalized_error = std::numeric_limits<double>::infinity();
    KDL::ChainFkSolverPos_recursive fk_solver(chain_);
    KDL::ChainJntToJacSolver jacobian_solver(chain_);
    KDL::JntArray kdl_joint_pos(JOINT_DOF_);
    KDL::Jacobian jacobian(JOINT_DOF_);
    const KDL::Frame target_frame(
        KDL::Rotation::RPY(target_pose_6d[3], target_pose_6d[4], target_pose_6d[5]),
        KDL::Vector(target_pose_6d[0], target_pose_6d[1], target_pose_6d[2]));
    const double damping_squared = damping * damping;

    for (int iteration = 0; iteration < max_iterations; ++iteration)
    {
        kdl_joint_pos.data = joint_pos;

        KDL::Frame current_frame;
        const int fk_status = fk_solver.JntToCart(kdl_joint_pos, current_frame);
        if (fk_status < 0)
        {
            return std::make_tuple(fk_status, best_joint_pos);
        }

        // KDL::diff computes a Cartesian displacement with an angle-axis
        // rotation error, avoiding discontinuities in direct RPY subtraction.
        const KDL::Twist pose_error = KDL::diff(current_frame, target_frame);
        Eigen::Matrix<double, 6, 1> error;
        for (int axis = 0; axis < 3; ++axis)
        {
            error[axis] = pose_error.vel[axis];
            error[axis + 3] = pose_error.rot[axis];
        }

        const double position_error = error.head<3>().norm();
        const double orientation_error = error.tail<3>().norm();
        const double normalized_error =
            position_error / position_tolerance + orientation_error / orientation_tolerance;
        if (normalized_error < best_normalized_error)
        {
            best_normalized_error = normalized_error;
            best_joint_pos = joint_pos;
        }

        if (position_error <= position_tolerance && orientation_error <= orientation_tolerance)
        {
            return std::make_tuple(KDL::SolverI::E_NOERROR, joint_pos);
        }

        const int jacobian_status = jacobian_solver.JntToJac(kdl_joint_pos, jacobian);
        if (jacobian_status < 0)
        {
            return std::make_tuple(jacobian_status, best_joint_pos);
        }

        // Scale the task-space error and Jacobian rows by error_weight so
        // position vs. orientation correction can be prioritized; this is
        // equivalent to solving the weighted least-squares problem
        // min ||W(e - J dq)||^2 + damping^2 ||dq||^2 via the task-space
        // identity dq = Jw^T (Jw Jw^T + lambda^2 I)^-1 (W e), Jw = W J.
        const Eigen::Matrix<double, 6, 1> weighted_error = error_weight.cwiseProduct(error);
        const Eigen::MatrixXd weighted_jacobian = error_weight.asDiagonal() * jacobian.data;

        // J^T (J J^T + lambda^2 I)^-1 e remains well-conditioned near
        // singularities and also supports redundant arms.
        Eigen::Matrix<double, 6, 6> damped_task_matrix =
            weighted_jacobian * weighted_jacobian.transpose() +
            damping_squared * Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 6, 1> task_update = damped_task_matrix.ldlt().solve(weighted_error);
        Eigen::VectorXd joint_step = weighted_jacobian.transpose() * task_update;
        if (!joint_step.allFinite())
        {
            return std::make_tuple(KDL::SolverI::E_NO_CONVERGE, best_joint_pos);
        }

        const double step_norm = joint_step.norm();
        if (step_norm > max_joint_step)
        {
            joint_step *= max_joint_step / step_norm;
        }
        joint_pos = (joint_pos + joint_step).cwiseMax(JOINT_POS_MIN_).cwiseMin(JOINT_POS_MAX_);
    }

    return std::make_tuple(KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED, best_joint_pos);
}

} // namespace arx
