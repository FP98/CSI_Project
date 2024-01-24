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

% Real params
s_param_r.J = s_param.J + std_dev.J*randn(1,1);
s_param_r.m = s_param.m + std_dev.m*randn(1,1);
s_param_r.b = s_param.b + std_dev.b*randn(1,1);
s_param_r.beta= s_param.beta + std_dev.beta*randn(1,1);
s_param_r.l = s_param.l + std_dev.l*randn(1,1);
%% Input parameters

std_dev.fm = 500;                           % fm standard deviation
std_dev.fa = 10;                             % fa standard deviation 

% Ideal parameters of the actuators transfer functions
Km1 = 1.1;      % [N] Gain fm
Km2 = 1.1;      % [N] Gain fa
T1 = 0.05;      % [s] Delay fm
T2 = 0.15;      % [s] Delay fa
Tm1 = 1;        % [s] Time constant fm
Tm2 = 1;        % [s] Time constant fa

% Std dev parameters actuator transfer function
std_dev_Km = 0.01;      % [N]
std_dev_T = 0.01;       % [s]
std_dev_Tm = 0.01;      % [s]

% Real parameters of the actuators transfer functions
Km1_real = Km1 + std_dev_Km*randn(1,1);     % [N] Gain fm
Km2_real = Km2 + std_dev_Km*randn(1,1);     % [N] Gain fa
T1_real = T1 + std_dev_T*randn(1,1);        % [s] Delay fm
T2_real = T2 + std_dev_T*randn(1,1);        % [s] Delay fa
Tm1_real = Tm1 + std_dev_Tm*randn(1,1);     % [s] Time constant fm
Tm2_real = Tm2 + std_dev_Tm*randn(1,1);     % [s] Time constant fa

%% Linear sys

% Working point
z_w = 400;
theta_w = 0;
dz_w = 0;
dtheta_w = 0;

x_w = [z_w; theta_w; dz_w; dtheta_w];

fm_w = s_param.g*s_param.m;
fa_w = 0;
%% System representation

% x = [z;theta;dz;dtheta]
A = [0 0 1 0;...
    0 0 0 1;...
    0 -fm_w*sin(theta_w)/s_param.m -s_param.b/s_param.m 0;...
    0 0 0 -s_param.beta/s_param.J];

% u = [fm;fa]
B = [0 0;...
    0 0;...
    cos(theta_w)/s_param.m 0;
    0 2*s_param.l/s_param.J];

C = [1 0 0 0; 0 1 0 0];
D = zeros(2,2);
%% Uncertant model

% Inizialise s
s = tf('s');

% Uncertant param system
J = ureal('J', s_param.J, 'PlusMinus', [-std_dev.J, std_dev.J]); 
m = ureal('m', s_param.m, 'PlusMinus', [-std_dev.m, std_dev.m]);
b = ureal('b', s_param.b, 'PlusMinus', [-std_dev.b, std_dev.b]);
beta = ureal('beta', s_param.beta, 'PlusMinus',[-std_dev.beta, std_dev.beta]);
l = ureal('l', s_param.l, 'PlusMinus',[-std_dev.l, std_dev.l]);

% System tranfer function
G1 = [1/(s*(b + m*s)) 0;...
    0 (2*l)/(s*(beta + J*s))];

% Input uncertanties param
Km1_u = ureal('Km1', Km1, 'PlusMin', [-std_dev_Km, std_dev_Km]); 
Km2_u = ureal('Km2', Km2, 'PlusMin', [-std_dev_Km, std_dev_Km]);
T1_u = ureal('T1', T1, 'PlusMin', [-std_dev_T, std_dev_T]);
T2_u = ureal('T2', T2, 'PlusMin',[-std_dev_T, std_dev_T]);
Tm1_u = ureal('Tm1', Tm1, 'PlusMin',[-std_dev_Tm, std_dev_Tm]);
Tm2_u = ureal('Tm2', Tm2, 'PlusMin',[-std_dev_Tm, std_dev_Tm]);

% Input tf
Gm1 = Km1_u*(1/(T1_u/2*s+1))*(1/(Tm1_u*s+1));
Gm2 = Km2_u*(1/(T2_u/2*s+1))*(1/(Tm2_u*s+1));

% G_u(s) global
Gu = G1*blkdiag(Gm1,Gm2);
G = Gu.NominalValue;
%% Weights of parametric uncertanity
% f = 1/(1e3*s+1)^2;
% WU_computation;
lm1 = tf([0 0 9.1135e-07 6.2827e-15],[1 0.0020 1.0000e-06 3.4728e-16]); % actuators 0.01 gamma 200
lm2 = tf([0 0 1.4226e-06 2.6489e-19],[1 0.0020 1.0000e-06 2.6472e-21]);

% lm1 = tf([0 0 5.3347e-07 1.1820e-15],[1 0.0020 1.0000e-06 5.2517e-17]); %actuators 0.01 gamma 104
% lm2 = tf([0 0 4.1362e-07 1.3081e-17],[1 0.0020 1.0000e-06 4.2159e-19]);
L = blkdiag(lm1,lm2);

%% G worst case
Gwc = G*(eye(2) + L);

%% Weights definitions
M = 1.5;
A_ =1e-4;
wb = 10; 

% Low pass filters
w1_11 = tf([1/M wb],[1 wb*A_]);
w1_22 = tf([1/M wb],[1 wb*A_]);

% Weight matrix
W1 = blkdiag(w1_11,w1_22);
W2 = blkdiag(tf(0.01), tf(0.01));

%% Connect P-K

% Input and output of G
G.u = 'u1';
G.y = 'y1';

% Input and output of W1
W1.u = 'e';
W1.y = 'z1';

% Input and output of W2
W2.u = 'u';
W2.y = 'z2';

% Input and output of L
L.u = 'u';
L.y = 'ydelta';
% Sum nodes
S1 = sumblk('e = r - y',2);
S2 = sumblk('u1 = u + d + udelta',2);
S3 = sumblk('y = y1 + n',2);      

% Connect
P = connect(G, W1, W2, L, S1, S2, S3, {'udelta','r', 'd', 'n', 'u'}, {'ydelta','z1', 'z2', 'e'});

%% Hyinf syntesis
opts = hinfsynOptions('Method','RIC');
[K,CL, gamma] = hinfsyn(P, 2, 2, opts);
Kss = K;                                    % Controller in state space form
K = tf(ss(K.A,K.B,K.C,K.D));                % Controller in Laplace domanin