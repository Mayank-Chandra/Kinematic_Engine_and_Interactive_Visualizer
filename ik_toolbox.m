clc;

% Rebuilding the compiled tree environment

robot = rigidBodyTree("DataFormat","column")

body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint1,[1,0,0,0],'dh');
body1.Joint = joint1;
addBody(robot,body1,robot.BaseName);

body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2,[1,0,0,0],"dh");
body2.Joint = joint2;
addBody(robot,body2,'link1');

body3 = rigidBody('link3');
joint3 = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint3,[1,0,0,0],"dh");
body3.Joint = joint3;
addBody(robot,body3,'link2');

% Define Desired Workspace Target Target Pose Matrix

% TargetCordinate x=1.3837 y=2.4319

T_target = eye(4);
T_target(1:3,1:3) = [0.2588,-0.9659,0;
    0.9659 0.2588 0;
    0 0 1];
T_target(1:3,4) = [1.3837; 2.4319; 0.0000];

ik_solver = inverseKinematics('RigidBodyTree',robot);

% Enforce Restrictions via weighting: [Roll,Pitch,Yaw,X,Y,Z]

weights = [0,0,1,1,1,0];
initial_guess = [0.0;0.0;0.0];

disp('Numerical Tracking ....');
[q_solution,info] = ik_solver('link3',T_target,weights,initial_guess);

disp('Numerical Inverse Kinematics Solution found:');
disp(q_solution);

fprintf('Verification Evaluation Matrix:\n');
fprintf('Status Flag:%\n',info.Status);
fprintf('Total Optimizer Interation:%d\n',info.Iterations);

q_deg = rad2deg(q_solution);
fprintf('Calculated Joint Profiles: [q1:%.2f deg,q2:%.2f deg,q3:%.2f deg]\n', ...
    q_deg(1),q_deg(2),q_deg(3));