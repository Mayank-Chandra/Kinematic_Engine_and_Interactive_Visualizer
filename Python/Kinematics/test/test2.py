import numpy as np
from Kinematics.transform import Transform

print('===Python Verification===')

q0 = np.array([1.0,0.0,0.0,0.0])
q1 = np.array([0.70710678,0.0,0.0,0.70710678])

q_mid = Transform.slerp(q0,q1,0.5)

print(f'SLERP Midpoint(t=0.5)->[w:{q_mid[0]:.4f},x:{q_mid[1]:.4f},y:{q_mid[2]:.4f},z:{q_mid[3]:.4f}]')