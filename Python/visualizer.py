import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from Kinematics.chain import DHTableSolver

'1. Instantiate the structural configurations (3-DOF,Lenght = 1.0,1.0,1.0)'

robot_dh_setup = [
    {'a':1.0,'alpha':0.0,'d':0},
    {'a':1.0,'alpha':0.0,'d':0},
    {'a':1.0,'alpha':0.0,'d':0},
]

solver = DHTableSolver(robot_dh_setup)

'2. Configure Figure Windows Properties'

fig,ax = plt.subplots(figsize=(7,7))
plt.subplots_adjust(bottom=0.35)
ax.set_xlim([-3.5 ,3.5])
ax.set_ylim([-3.5 ,3.5])
ax.grid(True)
ax.set_title('3DOF Planar Robot Wireframe')

line, = ax.plot([],[],'-o',lw=4,markersize=10,color='blue')

def update_skeleton(q1,q2,q3):

    q = np.array([q1,q2,q3])
    p0 = np.array([0.0,0.0])

    T1 = solver.compute_dh_matrix(q[0],robot_dh_setup[0]['d'],robot_dh_setup[0]['a'],robot_dh_setup[0]['alpha'])
    p1 = T1[:2,3]

    T2 = T1 @ solver.compute_dh_matrix(q[1],robot_dh_setup[1]['d'],robot_dh_setup[1]['a'],robot_dh_setup[1]['alpha'])
    p2 = T2[:2,3]

    T3 = T2 @ solver.compute_dh_matrix(q[2],robot_dh_setup[2]['d'],robot_dh_setup[2]['a'],robot_dh_setup[2]['alpha'])
    p3 = T3[:2,3]

    x_coord = [p0[0],p1[0],p2[0],p3[0]]
    y_coord = [p0[1],p1[1],p2[1],p3[1]]

    line.set_data(x_coord,y_coord)
    fig.canvas.draw_idle()

'3. GUI Placement Slider'

# FIX: each slider gets its own unique axes position (different 'bottom' value)
ax_q1 = plt.axes([0.15, 0.22, 0.7, 0.03])
ax_q2 = plt.axes([0.15, 0.15, 0.7, 0.03])
ax_q3 = plt.axes([0.15, 0.08, 0.7, 0.03])

slider_q1 = Slider(ax_q1,'Joint 1',-np.pi,np.pi,valinit=np.radians(30))
slider_q2 = Slider(ax_q2,'Joint 2',-np.pi,np.pi,valinit=np.radians(45))
slider_q3 = Slider(ax_q3,'Joint 3',-np.pi,np.pi, valinit=0.0)

def on_slider_move(val):
    update_skeleton(slider_q1.val,slider_q2.val,slider_q3.val)

slider_q1.on_changed(on_slider_move)
slider_q2.on_changed(on_slider_move)
slider_q3.on_changed(on_slider_move)

update_skeleton(np.radians(30),np.radians(45),0.0)
plt.show()