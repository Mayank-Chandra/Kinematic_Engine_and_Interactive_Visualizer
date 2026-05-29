clc; clear; close all;

robot = rigidBodyTree('DataFormat','column');
body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint1,[1,0,0,0],"dh");
body1.Joint = joint1;
addBody(robot, body1, robot.BaseName);

body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint2,[1,0,0,0],"dh");
body2.Joint = joint2
addBody(robot,body2,"link1");

body3 = rigidBody('link3')
joint3 = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint3,[1,0,0,0],"dh");
body3.Joint = joint3;
addBody(robot,body3,"link2");

% Setup Figure and Initialize Toolbox Display Canvas

fig = figure('Name','Robot Manipulator Dashboard','NumberTitle','off');
ax = axes(fig);

q_current = [pi/6;pi/4;0];
show(robot,q_current,'Parent',ax,'PreservePlot',false);
axis(ax,[-3.5 3.5 -3.5 3.5 -1 1]);
grid on;

uicontrol('Parent',fig,'Style','text','Position',[20 80 100 20],'String','Joint 1 Angle');
slider1 = uicontrol('Parent',fig,'Style','slider','Min',-pi,'Max',pi, ...
    'Value',q_current(1),'Position',[120 50 200 20]);

uicontrol('Parent',fig,'Style','text','Position',[20 50 100 20],'String','Joint 2 Angle');
slider2 = uicontrol('Parent',fig,'Style','slider','Min',-pi,'Max',pi, ...
    'Value',q_current(2),'Position',[120 50 200 20]);

uicontrol('Parent', fig, 'Style', 'text', 'Position', [20 20 100 20], 'String', 'Joint 3 Angle');
slider3 = uicontrol('Parent', fig, 'Style', 'slider', 'Min', -pi, 'Max', pi, ...
    'Value', q_current(3), 'Position', [120 20 200 20]);


% Continouos Real-Time UI Query Update Loop
fprintf('Running live Rendering loop');
while idx(fig)
    q1 = slider1.Value;
    q2 = slider2.Value;
    q3 = slider3.Value;

    show(robot,[q1;q2;q3],'Parent',ax,PreservePlot=false);
    axis(ax,[-3.5 3.5 -3.5 3.5 -1 1]);
    view(ax,2);
    drawnow;
end

function active = idx(fig_handle)
    active = isvalid(fig_handle)
end