%% Test animation
clc, close all;

%% Get data from simulink

% Real state
z = out.z.Data;
theta = out.theta.Data; 

% Resize
step = 100;
z_plot = z(1:step:end);
theta_plot = theta(1:step:end);
%% Plot

a = figure;

for i = 1:1:length(z_plot)
    clf;

% Draw Convertiplane
    draw_sys(150, z_plot(i), theta_plot(i), s_param, z_0, z_w);
    
% MATLAB optimization    
    drawnow limitrate;
    pause(0.01);
end