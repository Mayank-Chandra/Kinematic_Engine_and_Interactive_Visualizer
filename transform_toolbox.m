clc; clear;

% 1. Axis-Angle to Rotation Matrix

test_axis = [0,0,1];
test_theta = pi/2;
axang = [test_axis,test_theta];

R = axang2rotm(axang)
disp('1.Generated SO(3) Rotation Matrix (90 deg around Z:');
disp(R);

%2. Rotation Matrix to Axis-Angle

extracted_axang = rotm2axang(R);
disp('2.Extracted Axis-Angle Representation:');
disp(extracted_axang);

%3. Rotation matrix to Quaternion
q = rotm2quat(R);
disp('3.Generated Quaternion Representation:');
disp(q);
