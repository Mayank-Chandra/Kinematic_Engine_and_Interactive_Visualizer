import numpy as np

from Kinematics.jacobian import VelocityEngine

print("PYTHON JACOBIAN TRACK: VELOCITY ENGINE TEST")

q_test = np.array([np.radians(30),np.radians(45),np.radians(0)])

J = VelocityEngine.compute_planar_jacobian(q_test)

print("Calculated 3x3 Planar Jacobian Matrix:")

det_J = np.linalg.det(J)

print(f"\nPlanar Jacobian Matrix Determinant: {det_J:.4f}")