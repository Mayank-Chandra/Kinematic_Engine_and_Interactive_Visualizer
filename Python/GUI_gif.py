import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from Kinematics.chain import DHTableSolver

print("==================================================")
print("   PYTHON DYNAMIC SOLVER: GENERATING TIMELAPSE    ")
print("==================================================")

# 1. Instantiate the kinematics math model matching your project
robot_dh_setup = [
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0},
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0},
    {'a': 1.0, 'alpha': 0.0, 'd': 0.0}
]
solver = DHTableSolver(robot_dh_setup)

# 2. Setup Figure Elements
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim([-3.5, 3.5])
ax.set_ylim([-3.5, 3.5])
ax.grid(True)
ax.set_title("Robot Arm Trajectory Sweep")

# Draw the initial empty line skeleton
line, = ax.plot([], [], '-o', lw=4, markersize=10, color='blue')

# 3. Define a mathematical trajectory path for the simulation loop
# We will create 120 frames of smooth sinusoidal transitions
num_frames = 120
t = np.linspace(0, 2 * np.pi, num_frames)

# Generate joint movements over time using basic waveforms
q1_trajectory = np.sin(t) * np.radians(60)       # Joint 1 sweeps +/- 60 degrees
q2_trajectory = np.cos(t) * np.radians(45)       # Joint 2 sweeps +/- 45 degrees
q3_trajectory = np.sin(2 * t) * np.radians(30)   # Joint 3 moves at twice the frequency

def init():
    """Initializes the animation canvas background."""
    line.set_data([], [])
    return line,

def update(frame):
    """Calculates forward kinematics and updates plot data for frame index."""
    # Pull specific joint positions for the current time frame
    q1 = q1_trajectory[frame]
    q2 = q2_trajectory[frame]
    q3 = q3_trajectory[frame]
    
    # Core mathematical forward kinematics steps (exactly like Milestone 4)
    p0 = np.array([0.0, 0.0])
    
    T1 = solver.compute_dh_matrix(q1, robot_dh_setup[0]['d'], robot_dh_setup[0]['a'], robot_dh_setup[0]['alpha'])
    p1 = T1[:2, 3]
    
    T2 = T1 @ solver.compute_dh_matrix(q2, robot_dh_setup[1]['d'], robot_dh_setup[1]['a'], robot_dh_setup[1]['alpha'])
    p2 = T2[:2, 3]
    
    T3 = T2 @ solver.compute_dh_matrix(q3, robot_dh_setup[2]['d'], robot_dh_setup[2]['a'], robot_dh_setup[2]['alpha'])
    p3 = T3[:2, 3]
    
    x_coords = [p0[0], p1[0], p2[0], p3[0]]
    y_coords = [p0[1], p1[1], p2[1], p3[1]]
    
    # Update the visual line coordinates on screen
    line.set_data(x_coords, y_coords)
    return line,

# 4. Compile the animation framework
# fps=30 means 30 frames per second. 120 frames total = 4 seconds runtime.
ani = FuncAnimation(fig, update, frames=num_frames, init_func=init, blit=True)

# 5. Export directly to a production-grade file
output_filename = "demo.gif"
print(f"Compiling frame matrices into {output_filename}... please wait.")
ani.save(output_filename, writer='pillow', fps=30)
print("Export complete! You can open your directory to view your new gif file.")