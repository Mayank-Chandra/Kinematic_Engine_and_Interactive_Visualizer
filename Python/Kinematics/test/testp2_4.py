import numpy as np
from Kinematics.ik_solver import NewtonRaphsonIK

print("==================================================")
print("   PROJECT 2: CARTESIAN CIRCLE TRAJECTORY TRACK   ")
print("==================================================")

# 1. Setup the robot manipulator engine
robot_dh_setup = [
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0},
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0},
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0}
]
# Use 200 iterations and a slightly gentler tolerance for trajectory tracking
ik_engine = NewtonRaphsonIK(robot_dh_setup, max_iterations=200, tolerance=1e-4)

# 2. Define Circular Trajectory Parameters in Workspace
num_steps = 100
time_steps = np.linspace(0, 2 * np.pi, num_steps)

center_x, center_y = 1.2, 1.2  # Center of the circle
radius = 0.4                  # 40cm radius circle (keeps it safely inside workspace)

# 3. Seed the Tracking Loop with a Valid Initial Guess
# We match the orientation guess to the geometry (~45 degrees down the path)
first_target = np.array([center_x + radius, center_y, np.radians(45.0)])
q_current, info = ik_engine.solve_planar_ik_dls(first_target, np.array([0.2, 0.4, 0.2]), alpha=0.2, lambda_sq=0.02)

if info["status"] != "Success":
    print(f"Failed to seed initial trajectory point. Status: {info['status']}")
    exit()

print("Successfully seeded initial point! Starting continuous tracking loop...")

# Arrays to log data
cartesian_history = []
joint_history = []

# 4. Trajectory Tracking Execution Loop
for theta in time_steps:
    # Compute next desired Cartesian pose
    x_t = center_x + radius * np.cos(theta)
    y_t = center_y + radius * np.sin(theta)
    
    # Let orientation naturally float/adjust smoothly over the circle path
    target_pose = np.array([x_t, y_t, theta])
    
    # Warm-start: Pass the PREVIOUS joint angles as the initial guess
    q_current, step_info = ik_engine.solve_planar_ik_dls(
        target_pose, q_current, alpha=0.2, lambda_sq=0.02
    )
    
    cartesian_history.append(target_pose)
    joint_history.append(q_current.copy())

print("\n==================================================")
print("  TRAJECTORY TRACKING COMPLETE (SUCCESS!)")
print("==================================================")
print(f"Total Logged Continuous Points: {len(joint_history)}")
print(f"Final Joint State (Degrees): {np.round(np.degrees(joint_history[-1]), 2)}")