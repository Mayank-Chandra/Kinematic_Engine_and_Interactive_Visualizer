import numpy as np
import numpy.typing as npt
from typing import Tuple,List,Dict,Any
from Kinematics.chain import DHTableSolver
from Kinematics.jacobian import VelocityEngine


class NewtonRaphsonIK:
    
    def __init__(self,dh_param:list,max_iterations:int=100,tolerance:float=1e-6):
        "Initializes the numerical solver tracking options."
        
        self.fk_solver = DHTableSolver(dh_param)
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.dh_param = dh_param
        
        
    def solver_planar_ik(self,x_target:npt.NDArray[np.float64],q_intial:npt.NDArray[np.float64],alpha:float=0.5)->Tuple[npt.NDArray[np.float64],Dict[str,Any]]:
        
        """
            Executes the iterative Newton-Raphson optimizer search.
        """
        
        q_k = q_intial.copy().astype(np.float64)
        
        info = {
            "status":"Failed",
            "iterations":0,
            "error_history":[]
        }
        
        for i in range(self.max_iterations):
            info["iterations"]+=1
            
            #1 Compute current Forward Kinematics pose matrix
            
            T_curr = self.fk_solver.forward_kinematics(q_k)
            
            # Extract current planar position(x,y)
            
            x_curr = T_curr[0,3]
            y_curr = T_curr[1,3]
            
            phi_curr = q_k[0]+q_k[1]+q_k[2]
            
            state_curr = np.array([x_curr,y_curr,phi_curr])
            
            #2 Evaluate the task spacae error vector
            
            error_vector = x_target-state_curr
            
            error_vector[2]=(error_vector[2]+np.pi)%(2*np.pi)-np.pi
            
            error_norm = float(np.linalg.norm(error_vector))
            info['error_history'].append(error_norm)
            
            if error_norm<self.tolerance:
                info["status"]="Success"
                break
            
            #3 Compute Jacobian and Standard Inverse
            
            J = VelocityEngine.compute_planar_jacobian(
                q_k,self.dh_param[0]['a'],self.dh_param[1]['a'],self.dh_param[2]['a']
            )
            
            try:
                J_pinv = np.linalg.pinv(J)
            except np.linalg.LinAlgError:
                info['status'] = "Singular Matrix Crash"
                break            
                    
            #4 Project error into joint veloctiy steps
            
            delta_q = J_pinv @ error_vector
            
            q_k += alpha * delta_q
        return q_k,info
        
    def solve_planar_ik_dls(self,x_target:npt.NDArray[np.float64],q_intial:npt.NDArray[np.float64],alpha:float = 0.5,lambda_sq:float=0.1)-> Tuple[npt.NDArray[np.float64],Dict[str,Any]]:
        
        """
            Damped Least Squares(DLS) tracking for singularity robustness.
            Utilizes a damping factor (lambda_sq) to prevent infinte joint velocities
        """
        
        q_k = q_intial.copy().astype(np.float64)
        
        info = {"status":'Failed','iterations':0,'error_history':[]}
        
        for i in range(self.max_iterations):
            info['iterations']+=1
            
            #1 Forward Kinematics and Error
            
            T_curr = self.fk_solver.forward_kinematics(q_k)
            phi_curr = q_k[0] + q_k[1] + q_k[2]
            x_curr = np.array([T_curr[0, 3], T_curr[1, 3], phi_curr], dtype=np.float64)
            
            error = x_target-x_curr
            error[2] = (error[2]+np.pi)%(2*np.pi)-np.pi
            
            error_norm = float(np.linalg.norm(error))
            info["error_history"].append(error_norm)
            
            if error_norm < self.tolerance:
                info["status"] = 'Success'
                break
            
            #2 Jacobian Matrix
            
            J = VelocityEngine.compute_planar_jacobian(
                q_k,self.dh_param[0]['a'],self.dh_param[1]['a'],self.dh_param[2]['a']
            )
            #3 Damped Least Square Inversion
            I_3 = np.eye(3)
            J_dls = J.T @ np.linalg.inv(J @ J.T + lambda_sq*I_3)
            
            #4 Step Update
            q_k += alpha*(J_dls @ error)
            
        return q_k,info
    