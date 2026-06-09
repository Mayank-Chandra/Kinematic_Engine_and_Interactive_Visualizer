import numpy as np
import numpy.typing as npt

class VelocityEngine:
    @staticmethod
    
    def compute_planar_jacobian(q:npt.NDArray[np.float64],a1:float=1.0,a2:float=1.0,a3:float=1.0)->npt.NDArray[np.float64]:
        
        "Compute the analytical 3x3 geometric Jacobian for a 3-DOF planar manipulator"
        "Inputs: q as a shape (3,),array of joint angle [q1,q2,q3] in radians."
        
        s1 = np.sin(q[0])
        c1 = np.cos(q[0])
        
        s12 = np.sin(q[0]+q[1])
        c12 = np.cos(q[0]+q[1])
        
        s123 = np.sin(q[0]+q[1]+q[2])
        c123 = np.cos(q[0]+q[1]+q[2])
        
        
        J = np.array([
            [-a1*s1-a2*s12-a3*s123, -a2*s12-a3*s123,-a3*s123],
            [a1*c1+a2*c12+a3*c123,a2*c12+a3*c123,a3*c123],
            [1.0,1.0,1.0]
        ],dtype=np.float64)
        
        return J
    
