clc; clear; close all;

% ── Build 3-DOF Rigid Body Tree ──────────────────────────────────────────────
robot = rigidBodyTree('DataFormat','column');

body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint1,[1,0,0,0],"dh");
body1.Joint = joint1;
addBody(robot, body1, robot.BaseName);

body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2,[1,0,0,0],"dh");
body2.Joint = joint2;
addBody(robot, body2, "link1");

body3 = rigidBody('link3');
joint3 = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint3,[1,0,0,0],"dh");
body3.Joint = joint3;
addBody(robot, body3, "link2");

% ── Figure & Axes ─────────────────────────────────────────────────────────────
fig = figure('Name','Robot Manipulator Dashboard','NumberTitle','off', ...
             'Position',[100 100 700 600]);
ax = axes(fig, 'Position',[0.08 0.22 0.88 0.74]);

q_current = [pi/6; pi/4; 0];
show(robot, q_current, 'Parent', ax, 'PreservePlot', false);
axis(ax, [-3.5 3.5 -3.5 3.5 -1 1]);
grid on;
title(ax, '3-DOF Planar Arm — Live Kinematics');

% ── Slider Panel ──────────────────────────────────────────────────────────────
%   Layout (pixel coords from bottom-left of figure):
%   Row 1  y=100  → Joint 1 label + slider
%   Row 2  y= 65  → Joint 2 label + slider
%   Row 3  y= 30  → Joint 3 label + slider

uicontrol('Parent',fig,'Style','text', 'Position',[20 100 100 22], ...
          'String','Joint 1 (q1)','HorizontalAlignment','left');
slider1 = uicontrol('Parent',fig,'Style','slider', ...
          'Min',-pi,'Max',pi,'Value',q_current(1), ...
          'Position',[130 100 400 22]);

uicontrol('Parent',fig,'Style','text', 'Position',[20 65 100 22], ...
          'String','Joint 2 (q2)','HorizontalAlignment','left');
slider2 = uicontrol('Parent',fig,'Style','slider', ...
          'Min',-pi,'Max',pi,'Value',q_current(2), ...
          'Position',[130 65 400 22]);

uicontrol('Parent',fig,'Style','text', 'Position',[20 30 100 22], ...
          'String','Joint 3 (q3)','HorizontalAlignment','left');
slider3 = uicontrol('Parent',fig,'Style','slider', ...
          'Min',-pi,'Max',pi,'Value',q_current(3), ...
          'Position',[130 30 400 22]);

% ── Real-Time Render Loop ─────────────────────────────────────────────────────
fprintf('Live rendering loop started. Close the figure window to stop.\n');
while isWindowOpen(fig)
    q = [slider1.Value; slider2.Value; slider3.Value];
    show(robot, q, 'Parent', ax, 'PreservePlot', false);
    axis(ax, [-3.5 3.5 -3.5 3.5 -1 1]);
    view(ax, 2);
    drawnow;
end

% ── Helper ───────────────────────────────────────────────────────────────────
function active = isWindowOpen(fig_handle)
    active = isvalid(fig_handle);
end
