%% Initialization script for CSI simulation

%% Sample time of simulation
out_rate = 0.001;   %[s]
%% Ideal Initial conditions
z_0 = 0;
theta_0 = 0;
dz_0 = 0;
dtheta_0 = 0;
%% Ideal System params
s_param.J = 5000;     %[kgm^2]
s_param.m = 2000;     %[kg]
s_param.b = 150;      %[Ns/m]
s_param.beta = 15;    %[Nms/rad]
s_param.g = 9.81;     %[m/s^2]
s_param.l = 10;       %[m]