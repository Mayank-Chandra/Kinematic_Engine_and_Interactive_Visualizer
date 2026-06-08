clc; 


% Rebuilt the 3-DOF planar Structure

robot = rigidBodyTree('DataFormat','column');

body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint1,[1.0,0,0,0],'dh');
body1.Joint = joint1;
addBody(robot,body1,robot.BaseName);

body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2,[1,0,0,0],'dh');
body2.Joint = joint2;
addBody(robot,body2,'link1');

body3 = rigidBody('link3');
joint3 = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint3,[1,0,0,0],"dh");
body3.Joint = joint3;
addBody(robot,body3,'link2');


% Standard Test Config

q_test = [deg2rad(30); deg2rad(45); deg2rad(0)];

J_full = geometricJacobian(robot,q_test,'link3');
disp('Raw 6x3 Spatial Jacobian Matrix from Toolbox');
disp(J_full);

% Extract the 3x3 Planar Matrix Mapping 
J_planar = J_full([4,5,3],:);
disp(J_planar);

det_J = det(J_planar);

fprintf('Planar Jacobian Matrix Determinant: %.4f\n',det_J);