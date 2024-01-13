%% Initialization script for CSI simulation
clear
clc

%% Sample time of simulation

out_rate = 0.001;   %[s]

%% Sensors carateristics

% C state space matric
C = [1 0 0 0; 0 1 0 0];

% Sampling times
Ts.y1 = 0.03;       %[s]
Ts.y2 = 0.04;       %[s]

% Std deviation
std_dev.y1 = 0.07;      %[m]
std_dev.y2 = 0.09;  %[rad]

% Sensors covariance R
R = blkdiag(std_dev.y1^2, std_dev.y2^2);

%% Kalman filter

% Selector param
KALMAN_FILTER = true;       % True for KF, false for UKF

dt = 0.001;                 %[s] Sampling time of UKF
m_treshold = 5;

%% Ideal Initial conditions

z_0 = 0;
theta_0 = 0;
dz_0 = 0;
dtheta_0 = 0;

%% Std deviations initial conditions

std_dev.z_0 = 0.001;
std_dev.theta_0 = 0.001;
std_dev.dz_0 = 0.001;
std_dev.dtheta_0 = 0.001;
P_0 = zeros(4);%blkdiag(std_dev.z_0^2, std_dev.theta_0^2, std_dev.dz_0^2, std_dev.dtheta_0^2);

%% Ideal System params

s_param.J = 5000;     %[kgm^2]
s_param.m = 2000;     %[kg]
s_param.b = 150;      %[Ns/m]
s_param.beta = 15;    %[Nms/rad]
s_param.g = 9.81;     %[m/s^2]
s_param.l = 10;       %[m]

%% Input parameters

std_dev.fm = 0.01;                          % fm standard deviation
std_dev.fa = 0.01;                          % fa standard deviation 
Q = blkdiag(std_dev.fm^2, std_dev.fa^2);    % Disturbe process covariance
U_mean= [0; 0];                             % Mean disturbe process