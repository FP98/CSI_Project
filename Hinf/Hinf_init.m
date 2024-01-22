%% Initialization script for CSI simulation
clear
clc

CORRECTION = false;

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
std_dev.m = 50;        % [kg]
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

if CORRECTION
    % Shift eigenvalue from zero
    A_c = A - 0.01*eye(size(A));
    s = tf('s');
    lin_sys = ss(A_c,B,C,D);
    G1 = tf(lin_sys);
else
    % Nominal plant
    s = tf('s');
    lin_sys = ss(A,B,C,D);
    G1 = tf(lin_sys);
end

% Delay approximation
delay1= tf(exp(-T1*s));
delay2= tf(exp(-T2*s));
sys1 = pade(delay1,2);
sys2 = pade(delay2,2);

% Input transfer function
Gm1 = sys1*tf(Km1,[Tm1 1]);    
Gm2 = sys2*tf(Km2,[Tm2 1]);    

% Global plant (input + plant)
G = G1*blkdiag(Gm1,Gm2);
ss_global = ss(G); 

%% Weights definitions
M = 1.5;
A_ =1e-4;
wb = 100; 

% High pass filters
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

% Sum nodes
S1 = sumblk('e = r - y',2);
S2 = sumblk('u1 = u + d',2);
S3 = sumblk('y = y1 + n',2);      

% Connect
P = connect(G, W1, W2, S1, S2, S3, {'r', 'd', 'n', 'u'}, {'z1', 'z2', 'e'});

%% Hyinf syntesis
if CORRECTION 
    [K_pre,~, ~] = hinfsyn(P, 2, 2);
    
    % Inverse shift
    K_pre.A = K_pre.A + 0.01*eye(size(K_pre.A));
    K_non_min = ss(K_pre.A, K_pre.B, K_pre.C, K_pre.D);

    % Min real
    H = tf(K_non_min);
    K = ss(H, 'minimal');
    K = tf(K);
    Kss = ss(K);
else
    opts = hinfsynOptions('Method','RIC');
    [K,CL, gamma] = hinfsyn(P, 2, 2, opts);
    Kss = K;                                    % Controller in state space form
    K = tf(ss(K.A,K.B,K.C,K.D));                % Controller in Laplace domanin
end
%% MU-analysis
MU_analysis;