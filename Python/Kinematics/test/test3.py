import numpy as np
from Kinematics.chain import DHTableSolver

print('===Python Verification===')

robot_dh_setup = [
    {'a':1.0,'alpha':0.0,'d':0.0},
    {'a':1.0,'alpha':0.0,'d':0.0},
    {'a':1.0,'alpha':0.0,'d':0.0}
]

solver = DHTableSolver(robot_dh_setup)
q_test = np.array([np.radians(30),np.radians(45),np.radians(0)])

T_result = solver.forward_kinematics(q_test)

print('Result Pose Matrix at End Effector: ')
print(np.round(T_result,4))