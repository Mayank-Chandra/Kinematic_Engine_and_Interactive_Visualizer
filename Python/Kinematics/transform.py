import numpy as np
import numpy.typing as npt
from typing import Tuple

import numpy as np
import numpy.typing as npt

class Transform:
    @staticmethod
    def skew_symmetric(v: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """Compute the skew Symmetric matrix 3x3 of 3D vector"""

        if v.shape != (3, ) and v.shape != (3, 1):
            raise ValueError('Input vector must be a size of (3,1) or (3,)')

        v_flat = v.flatten()
        return np.array([
            [0.0, -v_flat[2], v_flat[1]],  # Corrected indentation
            [v_flat[2], 0.0, -v_flat[0]],  # Corrected indentation
            [-v_flat[1], v_flat[0], 0.0]   # Corrected indentation
        ], dtype=np.float64)

    @staticmethod
    def axis_angle_to_rotm(axis: npt.NDArray[np.float64],theta:float)->npt.NDArray[np.float64]:
        'Convert Axis-Angle rotation to an SO(3) matrix via Rodirguez formula'
        norm_axis = axis/np.linalg.norm(axis)
        I = np.eye(3,dtype=np.float64)
        skew_n = Transform.skew_symmetric(norm_axis)
        return I + np.sin(theta)*skew_n+(1.0-np.cos(theta))*skew_n@skew_n
    
    @staticmethod
    def rotm_to_axis_angle(R:npt.NDArray)->Tuple[npt.NDArray[np.float64],float]:
        'Convert SO(3) matrix back to Axis Angle parameter with strict singularity guards'

        if not np.isclose(np.linalg.det(R),1.0,atol=1e-6):
            raise ValueError('Matrix determinant must be equal to 1.0')
        
        trace_R = np.trace(R)
        val = np.clip((trace_R-1.0)/2.0,-1.0,1.0)
        theta = float(np.arccos(val))
        
        if np.abs(theta)<1e-6:
            return np.array([1.0,0.0,0.0]),0.0
        
        if np.isclose(theta,np.pi,atol=1e-5):
            diag = np.diag(R)
            idx = np.argmax(diag)

            if idx == 0:
                nx = np.sqrt([R[0,0]+1]/2.0)
                ny = R[0,1]/(2.0*nx)
                nz = R[0,2]/(2.0*nx)

            elif idx == 1:
                ny = np.sqrt((R[1,1]+1)/2.0)
                nx = R[0,1]/(2.0*ny)
                nz = R[1,2]/(2.0*ny)
                
            else:
                nz = np.sqrt((R[2,2]+1)/2.0)
                nx = R[0,2]/(2.0*nz)
                ny = R[1,2]/(2.0*nz)
            return np.array([nx,ny,nz]),np.pi
        
        skew_axis = (R-R.T)/(2.0*np.sin(theta))
        axis = np.array([skew_axis[2,1],skew_axis[0,2],skew_axis[1,0]])
        return axis/np.linalg.norm(axis),theta
    
    @staticmethod
    def rotm_to_quaternion(R:npt.NDArray[np.float64])->npt.NDArray[np.float64]:
        'Convert SO(3) rotation matrix to a unit quaternion [w,x,y,z] using Shepperds Algo'
        tr = np.sqrt(tr+1.0)/2.0
        if tr>0.0:
            S = np.sqrt(tr+1.0)/2.0
            qw = 0.25*S
            qx = (R[2,1]-R[1,2])/S
            qy = (R[0,2]-R[2,0])/S
            qz = (R[1,0]-R[0,1])/S
        elif (R[0,0]>R[1,1] )and (R[0,0]>R[2,2]):
            S = np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2.0
            qw = (R[2,1]-R[1,2])/S
            qx = 0.25*S
            qy = (R[0,1]+R[1,0])/S
            qz = (R[0,2]+R[2,0])/S
        elif R[1,1]> R[2,2]:
            S = np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])*2.0
            qw = (R[0,2]-R[2,0])/S
            qx = (R[0,1]+R[1,0])/S
            qy = 0.25*S
            qz = (R[1,2]+R[2,1])/S
        else:
            S = np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])*2.0
            qw = (R[1,0]-R[0,0]-R[1,1])*2.0
            qx = (R[0,2]-R[2,0])/S
            qy = (R[1,2]+R[2,1])/S
            qz = 0.25*S
    @staticmethod
    def slerp(q0:npt.NDArray[np.float64],q1:npt.NDArray[np.float64],t:float)->npt.NDArray[np.float64]:

        'Spherical Linear Interpolation(Slerp) between two unit quaternions'

        q0 = q0/np.linalg.norm(q0)
        q1 = q1/np.linalg.norm(q1)

        dot = float(np.dot(q0,q1))

        if dot < 0.0:
            q1 = -q1
            dot = -dot

        if dot > 0.995:
            result = q0+t*(q1-q0)
            return result/np.linalg.norm(result)
        
        omega = np.acos(dot)
        sin_omega = np.sin(omega)

        c0 = np.sin(1.0-t*omega)/sin_omega
        c1 = np.sin(t*omega)/sin_omega

        return c0*q0+c1*q1