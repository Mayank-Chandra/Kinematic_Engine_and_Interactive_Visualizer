clc;clear;

axis_vec = [0,0,2];
theta0 = 0.0;
theta1 = pi/2;

R0 = axang2rotm([axis_vec,theta0]);
R1 = axang2rotm([axis_vec,theta1]);

q0_obj = quaternion(R0,'rotmat','point');
q1_obj = quaternion(R1, 'rotmat', 'point');

t_steps = [0.0,0.5,1.0];

fprintf('Evaluating Toolbox Trajectory Loop:\n');
for i = 1:length(t_steps)
    t = t_steps;
    q_interp = slerp(q0_obj,q1_obj,t);

    [w,x,y,z] = parts(q_interp);
    fprintf('   t = %.2f -> Vector: [w: %.4f, x: %.4f, y: %.4f, z: %.4f]\n', ...
            t, w, x, y, z);
end
