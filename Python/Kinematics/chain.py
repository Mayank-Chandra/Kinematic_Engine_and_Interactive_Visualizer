import numpy as np
import numpy.typing as npt
from typing import List,Dict

class DHTableSolver:
    def __init__(self,dh_param:List[Dict[str,float]]):
        'Initialize seriel link chain parsing a list of geometric parameter dictionaries'
        self.dh_param = dh_param
        self.num_joints = len(dh_param)


    @staticmethod
    def compute_dh_matrix(theta:float,d:float,a:float,alpha:float)->npt.NDArray[np.float64]:
        'Compute a Standard 4x4 DH homogenous Transformation Matrix'
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array([
            [ct,-st*ca,st*sa,a*ct],
            [st,ct*ca,-ct*sa,a*st],
            [0.0,sa,ca,d],
            [0.0,0.0,0.0,1.]
        ],dtype=np.float64)
    
    def forward_kinematics(self,joint_angles:npt.NDArray[np.float64])->npt.NDArray[np.float64]:

        'Compute Total forward Kinematics mapping by sequentially post-mulitpyling frames'

        T_cum = np.eye(4,dtype=np.float64)

        for i in range(self.num_joints):
            q_i = joint_angles[i]
            param = self.dh_param[i]

            theta_i = q_i+param.get('theta_offset',0.0)
            d_i = param['d']
            a_i = param['a']
            alpha_i = param['alpha']

            T_local = DHTableSolver.compute_dh_matrix(theta_i,d_i,a_i,alpha_i)
            T_cum = T_cum@T_local

            return T_cum