clc; clear;

robot = rigidBodyTree('DataFormat','column');

body1 = rigidBody('Link1');
joint1 = rigidBodyJoint('joint1','revolute');

setFixedTransform(joint1,[1,0,0,0],'dh');
body1.Joint = joint1;
addBody(robot,body1,robot.BaseName);

body2 = rigidBody('Link2');
joint2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2,[1,0,0,0],'dh');
body2.Joint = joint2;
addBody(robot,body2,"Link1");

body3 = rigidBody('Link3');
joint3 = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint3,[1,0,0,0],"dh");
body3.Joint = joint3;
addBody(robot,body3,'Link2');

disp('Robot Structural Tree Architecture:')
showdetails(robot);

% Evaluate Forward Kinematics at a test Config
q_test = [deg2rad(30); deg2rad(45); deg2rad(0)];

T_end_effector = getTransform(robot,q_test,'Link3');
disp('Toolbox calculated Transformation Matrix at End-Effector');
disp(T_end_effector);
