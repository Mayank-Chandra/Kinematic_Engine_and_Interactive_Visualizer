import numpy as np
from Kinematics.transform import Transform

print('===Python Verification===')

'Test 180-degree singularity around Y-axis'

axis_test = np.array([0.0,1.0,0.0])
theta_test = np.pi

R = Transform.axis_angle_to_rotm(axis_test,theta_test)

ax_out,th_out = Transform.rotm_to_axis_angle(R)

print(f'Singularity Result -> Extracted Angle: {th_out:.4f} rad, Axis: {ax_out}')
