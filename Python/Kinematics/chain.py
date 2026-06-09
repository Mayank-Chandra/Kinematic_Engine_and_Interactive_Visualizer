import numpy as np
import numpy.typing as npt
from typing import List, Dict
class DHTableSolver:
    def __init__(self, dh_param: List[Dict[str, float]]):
        'Initialize serial link chain parsing a list of geometric parameter dictionaries'
        self.dh_param = dh_param
        self.num_joints = len(dh_param)
    @staticmethod
    def compute_dh_matrix(theta: float, d: float, a: float, alpha: float) -> npt.NDArray[np.float64]:
        'Compute a Standard 4x4 DH homogenous Transformation Matrix'
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0.0, sa,     ca,    d],
            [0.0, 0.0,    0.0,   1.0]
        ], dtype=np.float64)
    
    def forward_kinematics(self, joint_angles: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        'Compute Total forward Kinematics mapping by sequentially post-multiplying frames'
        T_cum = np.eye(4, dtype=np.float64)
        for i in range(self.num_joints):
            q_i = joint_angles[i]
            param = self.dh_param[i]
            theta_i = q_i + param.get('theta_offset', 0.0)
            d_i = param['d']
            a_i = param['a']
            alpha_i = param['alpha']
            T_local = DHTableSolver.compute_dh_matrix(theta_i, d_i, a_i, alpha_i)
            T_cum = T_cum @ T_local
        return T_cum
    
    def get_joint_positions(self, joint_angles: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """
        Computes the intermediate 3D Cartesian coordinates of all joint centers.
        Returns an array of shape (N+1, 3) representing [x, y, z] for:
        [Base, Joint 1, Joint 2, End-Effector]
        """
        positions = []
        T_cum = np.eye(4, dtype=np.float64)
        
        # Base origin P_0 is always at [0, 0, 0]
        positions.append(T_cum[0:3, 3])
        
        for i in range(self.num_joints):
            q_i = joint_angles[i]
            param = self.dh_param[i]
            
            theta_i = q_i + param.get('theta_offset', 0.0)
            d_i = param['d']
            a_i = param['a']
            alpha_i = param['alpha']
            
            T_local = DHTableSolver.compute_dh_matrix(theta_i, d_i, a_i, alpha_i)
            T_cum = T_cum @ T_local
            
            # Extract the current frame's origin position vector [x, y, z]
            positions.append(T_cum[0:3, 3])
            
        return np.array(positions, dtype=np.float64)
