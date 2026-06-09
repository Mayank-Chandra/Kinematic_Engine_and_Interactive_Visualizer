import numpy as np
from Kinematics.ik_solver import NewtonRaphsonIK

print("   PYTHON FROM-SCRATCH TRACK: DLS TORTURE TEST    ")

robot_dh_setup = [
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0},
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0},
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0}
]
ik_engine = NewtonRaphsonIK(robot_dh_setup,max_iterations=200,tolerance=1e-5)

# Target: Singularity (x=3.0, y=0.0 means fully straight arm)
target_pose = np.array([3.0, 0.0, 0.0])
q_guess = np.array([0.1, 0.1, 0.1])

print("Pushing robot to singular configuration boundary...")
q_sol, info = ik_engine.solve_planar_ik_dls(target_pose, q_guess, alpha=0.2, lambda_sq=0.2)

print(f"\nDLS Solver Status: {info['status']}")
print(f"Iterations Required: {info['iterations']}")
print("Resulting Joints (Degrees):")
print(np.round(np.degrees(q_sol), 2))