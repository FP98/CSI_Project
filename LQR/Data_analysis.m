%% Script used to analyse data from Simulink simulation
close all

% Get vector from Simulink
t = out.tout;
z = out.z.Data;
theta = out.theta.Data;
dz = out.dz.Data;
dtheta = out.dtheta.Data;

z = reshape(z,1,size(z,3));
theta = reshape(theta,1,size(theta,3));
dz = reshape(dz,1,size(dz,3));
dtheta = reshape(dtheta,1,size(dtheta,3));

% Find overshoot
ss_value = z(end);
overshoot = (max(z) - ss_value)/ss_value;

% Find setting time
max_value = z(end)*(1 + 0.05);          % Boundary evaluation
min_value = z(end)*(1 - 0.05);

st_candidates = find(z >= max_value | z <= min_value);       % Setting time candidates

setting_time_index = max(st_candidates);
setting_time = t(setting_time_index);

% Display value
disp("Overshoot: ");
disp(overshoot*100);

disp("Setting time:" );
disp(setting_time);

% Plot value
subplot(2,2,1)
hold on
plot(t,z, 'k', 'LineWidth',2)
plot(t,max_value*ones(size(t)), '--b')
plot(t,min_value*ones(size(t)), '--b')
plot([setting_time setting_time], [0 z(setting_time_index)], '--b')
plot(t,max(z)*ones(size(t)), '--k')
grid on
ylabel("z [m]")
xlabel("Time [s]")
title("Altitude")
hold off

subplot(2,2,2)
plot(t,theta, 'k', 'LineWidth',2)
xlabel("Time [s]")
title("Pitch")
ylabel("\theta [rad]")
grid on

subplot(2,2,3)
plot(t,dz, 'k', 'LineWidth',2)
xlabel("Time [s]")
title("Vertical speed ")
ylabel("dz [m/s]")
grid on

subplot(2,2,4)
plot(t,dtheta, 'k', 'LineWidth',2)
xlabel("Time [s]")
title("Pitch rate")
ylabel("d\theta [rad/s]")
grid on