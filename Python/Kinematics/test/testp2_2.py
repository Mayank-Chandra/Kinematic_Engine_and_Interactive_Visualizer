import numpy as np
from Kinematics.ik_solver import NewtonRaphsonIK

print("   PYTHON FROM-SCRATCH TRACK: STANDARD IK SOLVER  ")


robot_dh_setup = [
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0},
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0},
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0}
]

ik_engine = NewtonRaphsonIK(robot_dh_setup, max_iterations=100, tolerance=1e-6)

# Target: x = 1.3837, y = 2.4319, phi = 75 degrees
target_pose = np.array([1.3837, 2.4319, np.radians(75.0)])
q_guess = np.array([0.1, 0.1, 0.1])

print("Running numerical tracking convergence loop...")
q_sol, run_info = ik_engine.solver_planar_ik(target_pose, q_guess, alpha=0.5)

print(f"\nSolver Status: {run_info['status']}")
print(f"Iterations: {run_info['iterations']}")
print("\nConverted Joint Profiles (Degrees):")
print(np.round(np.degrees(q_sol), 2))
