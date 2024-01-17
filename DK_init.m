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
std_dev.y2 = 0.09;      %[rad]

% Sensors covariance R
R = blkdiag(std_dev.y1^2, std_dev.y2^2);

%% Initial conditions

% Ideal initial conditions
z_0 = 200;
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

%% System params

% Ideal param
s_param.J = 5000;     % [kgm^2]
s_param.m = 2000;     % [kg]
s_param.b = 150;      % [Ns/m]
s_param.beta = 15;    % [Nms/rad]
s_param.g = 9.81;     % [m/s^2]
s_param.l = 10;       % [m]

% Standard dev param
std_dev.J = 350;        % [kgm^2]
std_dev.m = 300;        % [kg]
std_dev.b = 10;         % [Ns/m]
std_dev.beta = 2;       % [m/s^2]
std_dev.l = 0.2;        % [m]

%% Input parameters

std_dev.fm = 500;                           % fm standard deviation
std_dev.fa = 10;                            % fa standard deviation 

% Ideal parameters of the actuators transfer functions
Km1 = 1.1;      % [N] 
Km2 = 1.1;      % [N] 
T1 = 0.05;      % [s] 
T2 = 0.15;      % [s] 
Tm1 = 1;        % [s] 
Tm2 = 1;        % [s] 

% Std dev parameters actuator transfer function
std_dev_Km = 0.01;      % [N]
std_dev_T = 0.01;       % [s]
std_dev_Tm = 0.01;      % [s]

%% Linear sys

% Working point
z_w = 400;
theta_w = 0;
dz_w = 0;
dtheta_w = 0;

x_w = [z_w; theta_w; dz_w; dtheta_w];

fm_w = s_param.g*s_param.m;
fa_w = 0;

%% Uncertant model

% Uncertant param system
J = ureal('J', s_param.J, 'PlusMinus', [-std_dev.J, std_dev.J]); 
m = ureal('m', s_param.m, 'PlusMinus', [-std_dev.m, std_dev.m]);
b = ureal('b', s_param.b, 'PlusMinus', [-std_dev.b, std_dev.b]);
beta = ureal('beta', s_param.beta, 'PlusMinus',[-std_dev.beta, std_dev.beta]);
l = ureal('l', s_param.l, 'PlusMinus',[-std_dev.l, std_dev.l]);

% State space matrix system
A = [0 0 1 0;...
    0 0 0 1;...
    0 -fm_w*sin(theta_w)/m -b/m 0;...
    0 0 0 -beta/J];

B = [0 0;...
    0 0;...
    cos(theta_w)/m 0;
    0 2*l/J];                   % u = [fm;fa]

C = [1 0 0 0;...
    0 1 0 0];

sys_u = tf(A,B,C,zeros(2,2));

% Input uncertanties param
Km1_u = ureal('Km1', Km1, 'PlusMin', [-std_dev_Km, std_dev_Km]); 
Km2_u = ureal('Km2', Km2, 'PlusMin', [-std_dev_Km, std_dev_Km]);
T1_u = ureal('T1', T1, 'PlusMin', [-std_dev_T, std_dev_T]);
T2_u = ureal('T2', T2, 'PlusMin',[-std_dev_T, std_dev_T]);
Tm1_u = ureal('Tm1', Tm1, 'PlusMin',[-std_dev_Tm, std_dev_Tm]);
Tm2_u = ureal('Tm2', Tm2, 'PlusMin',[-std_dev_Tm, std_dev_Tm]);

s = tf('s');

% Input tf
Gm1 = Km1_u*(1/(T1_u/2*s+1))*(1/(Tm1_u*s+1));
Gm2 = Km2_u*(1/(T2_u/2*s+1))*(1/(Tm2_u*s+1));

% % G_u(s) global
% G_u(1,1) = Gm1*G_u(1,1);
% G_u(2,2) = Gm2*G_u(2,2);

% sys_u_t = ss(G_u);

[M,Delta,~] = lftdata(sys_u);

%% Nominal model

% System nominal value
sys_n = usubs(G_u, 'J', s_param.J, 'm', s_param.m,'b', s_param.b, 'beta', s_param.beta,'l', s_param.l, 'Km1', Km1, 'Km2', Km2,'T1', T1, 'T2', T2, 'Tm1', Tm1, 'Tm2', Tm2);


