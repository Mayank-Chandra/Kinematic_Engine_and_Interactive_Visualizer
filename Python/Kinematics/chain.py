import numpy as np
import numpy.typing as npt
from typing import List, Dict


class DHTableSolver:

    def __init__(self, dh_params: List[Dict[str, float]]):
        """Initialize a serial-link chain from a list of DH parameter dicts.

        Each dict must contain keys: 'a', 'd', 'alpha'.
        Optional key: 'theta_offset' (default 0.0) for non-zero joint zero positions.

        Example (3-DOF planar, unit links):
            dh_params = [
                {'a': 1.0, 'd': 0.0, 'alpha': 0.0},
                {'a': 1.0, 'd': 0.0, 'alpha': 0.0},
                {'a': 1.0, 'd': 0.0, 'alpha': 0.0},
            ]
        """
        required = {'a', 'd', 'alpha'}
        for i, p in enumerate(dh_params):
            missing = required - p.keys()
            if missing:
                raise ValueError(f"DH parameter dict at index {i} is missing keys: {missing}")
        self.dh_params = dh_params
        self.num_joints = len(dh_params)

    def _validate_joint_angles(self, joint_angles: npt.NDArray[np.float64]) -> None:
        """Raise if joint_angles length does not match the chain."""
        if len(joint_angles) != self.num_joints:
            raise ValueError(
                f"Expected {self.num_joints} joint angles, got {len(joint_angles)}"
            )

    @staticmethod
    def compute_dh_matrix(
        theta: float,
        d: float,
        a: float,
        alpha: float
    ) -> npt.NDArray[np.float64]:
        """Compute the standard 4x4 DH homogeneous transformation matrix T_i^{i-1}."""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0, sa,       ca,      d     ],
            [0.0, 0.0,      0.0,     1.0   ]
        ], dtype=np.float64)

    def forward_kinematics(
        self,
        joint_angles: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Compute the end-effector pose T_n^0 by sequentially post-multiplying DH frames.

        Returns:
            4x4 homogeneous transformation matrix mapping base frame to end-effector.
        """
        self._validate_joint_angles(joint_angles)
        T = np.eye(4, dtype=np.float64)
        for i in range(self.num_joints):
            p = self.dh_params[i]
            theta_i = joint_angles[i] + p.get('theta_offset', 0.0)
            T = T @ DHTableSolver.compute_dh_matrix(theta_i, p['d'], p['a'], p['alpha'])
        return T

    def get_joint_positions(
        self,
        joint_angles: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Return Cartesian positions of all joint origins including base and end-effector.

        Returns:
            Array of shape (num_joints + 1, 3) — [base, joint_1, ..., end_effector].
        """
        self._validate_joint_angles(joint_angles)
        positions = []
        T = np.eye(4, dtype=np.float64)
        positions.append(T[0:3, 3].copy())
        for i in range(self.num_joints):
            p = self.dh_params[i]
            theta_i = joint_angles[i] + p.get('theta_offset', 0.0)
            T = T @ DHTableSolver.compute_dh_matrix(theta_i, p['d'], p['a'], p['alpha'])
            positions.append(T[0:3, 3].copy())
        return np.array(positions, dtype=np.float64)

    def get_all_transforms(
        self,
        joint_angles: npt.NDArray[np.float64]
    ) -> List[npt.NDArray[np.float64]]:
        """Return the full 4x4 transform at every frame, not just the positions.

        Returns:
            List of (num_joints + 1) arrays each of shape (4, 4):
            [T_base, T_0^1, T_0^2, ..., T_0^n]
        """
        self._validate_joint_angles(joint_angles)
        transforms = []
        T = np.eye(4, dtype=np.float64)
        transforms.append(T.copy())
        for i in range(self.num_joints):
            p = self.dh_params[i]
            theta_i = joint_angles[i] + p.get('theta_offset', 0.0)
            T = T @ DHTableSolver.compute_dh_matrix(theta_i, p['d'], p['a'], p['alpha'])
            transforms.append(T.copy())
        return transforms

    def __repr__(self) -> str:
        return f"DHTableSolver(num_joints={self.num_joints})"
