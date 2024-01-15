%% Initialization script for CSI simulation
clear
clc

%% Sample time of simulation

out_rate = 0.001;   %[s]

%% Sensors carateristics

% Sampling times
Ts.y1 = 0.03;       %[s]
Ts.y2 = 0.04;       %[s]

% Std deviation
std_dev.y1 = 0.07;      %[m]
std_dev.y2 = 0.09;  %[rad]

% Sensors covariance R
R = blkdiag(std_dev.y1^2, std_dev.y2^2);

%% Initial conditions

% Ideal initial conditions
z_0 = 0;
theta_0 = 0;
dz_0 = 0;
dtheta_0 = 0;

x_0 = [z_0; theta_0; dz_0; dtheta_0];

% Std deviations initial conditions
std_dev.z_0 = 0.001;
std_dev.theta_0 = 0.001;
std_dev.dz_0 = 0.001;
std_dev.dtheta_0 = 0.001;
P_0 = blkdiag(std_dev.z_0^2, std_dev.theta_0^2, std_dev.dz_0^2, std_dev.dtheta_0^2);

% Real initial condition
z_0_real = z_0+std_dev.z_0*randn(1,1);
theta_0_real = theta_0+std_dev.theta_0*randn(1,1);
dz_0_real = dz_0+std_dev.dz_0*randn(1,1);
dtheta_0_real = dtheta_0+std_dev.dtheta_0*rand(1,1);

x_0_real = [z_0_real; theta_0_real; dz_0_real; dtheta_0_real];

%% Ideal System params

s_param.J = 5000;     % [kgm^2]
s_param.m = 2000;     % [kg]
s_param.b = 150;      % [Ns/m]
s_param.beta = 15;    % [Nms/rad]
s_param.g = 9.81;     % [m/s^2]
s_param.l = 10;       % [m]

%% Input parameters

std_dev.fm = 500;                          % fm standard deviation
std_dev.fa = 1;                          % fa standard deviation 
Q = blkdiag(std_dev.fm^2, std_dev.fa^2);    % Disturbe process covariance
U_mean= [0; 0];                             % Mean disturbe process

%% Linear sys

% Working point
z_w = 20;
theta_w = 0;
dz_w = 0;
dtheta_w = 0;

x_w = [z_w; theta_w; dz_w; dtheta_w];

fm_w = s_param.g*s_param.m;
fa_w = 0;

% State space matrix
A = [0 0 1 0;...
    0 0 0 1;...
    -s_param.b/s_param.m -fm_w*sin(theta_w)/s_param.m 0 0;...
    0 0 0 -s_param.b/s_param.J];

B = [0 0;...
    0 0;...
    cos(theta_w) 0;
    0 2*s_param.l/s_param.J];                   % u = [fm;fa]

C = [1 0 0 0;...
    0 1 0 0];

D = zeros(2,2);

lin_sys = ss(A,B,C,D);

%% Kalman filter

% Selector param
KALMAN_FILTER = true;       % True for KF, false for UKF

% KF matrix
[~, Kf] = kalman(lin_sys, Q,R);

% UKF param
dt = 0.001;                 % [s] Sampling time of UKF
m_treshold = 5;