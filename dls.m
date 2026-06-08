% dls.m - Standalone Damped Least-Squares Demonstration
clc;
fprintf('--- DLS Standalone Demonstration ---\n');

% 1. Setup the singular configuration (Fully extended arm)
Q_singular = [0; 0; 0]; 
J_full = geometricJacobian(robot, Q_singular, 'link3');
J_planar = J_full([4, 5, 3], :);

% 2. Define a small movement error we want to achieve
error = [0.1; 0.1; 0.0]; 

% 3. The DLS "Safety Net" (Damping Factor)
% If lambda is 0, this acts exactly like inv(J).
% As lambda increases, the motion becomes safer but less precise.
lambda = 0.5; 

% 4. DLS Calculation (The robust inversion)
% J_dls = J' * inv(J*J' + lambda^2 * I)
I = eye(3);
J_dls = J_planar' / (J_planar * J_planar' + (lambda^2) * I);

% 5. Calculate the safe joint step
dq = J_dls * error;

fprintf('Determinant of J_planar: %.4f (Effectively Singular)\n', det(J_planar));
fprintf('Calculated Joint Step (dq) using DLS:\n');
disp(dq);
fprintf('--- Calculation Complete (No crash!) ---\n');
