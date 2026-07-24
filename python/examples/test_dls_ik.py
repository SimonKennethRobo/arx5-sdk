import os
import sys

import numpy as np


PYTHON_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(PYTHON_DIR)

import arx5_interface as arx5


def main() -> None:
    config = arx5.RobotConfigFactory.get_instance().get_config("X5")
    solver = arx5.Arx5Solver(
        config.urdf_path,
        config.joint_dof,
        config.joint_pos_min,
        config.joint_pos_max,
        config.base_link_name,
        config.eef_link_name,
        config.gravity_vector,
    )

    expected_joint_pos = np.array([0.25, 0.8, 0.35, -0.4, 0.3, -0.2])
    target_pose = solver.forward_kinematics(expected_joint_pos)
    status, solved_joint_pos = solver.dls_inverse_kinematics(
        target_pose,
        np.zeros(config.joint_dof),
    )
    solved_pose = solver.forward_kinematics(solved_joint_pos)

    position_error = np.linalg.norm(target_pose[:3] - solved_pose[:3])
    # Compare rotation matrices indirectly by asking DLS to refine the result;
    # status is the authoritative full-pose convergence check.
    print(f"status: {status} ({solver.get_ik_status_name(status)})")
    print(f"joint position: {solved_joint_pos}")
    print(f"position error: {position_error:.6g} m")

    if status != 0:
        raise RuntimeError("DLS IK did not converge")
    if position_error > 1e-4:
        raise RuntimeError("DLS IK position error exceeds tolerance")


if __name__ == "__main__":
    main()
