%% Nominal G(s) evaluation 

% Simbolic variables
syms s m b beta J l;

% Working point
theta_w = 0;
fm_w = s_param.g*s_param.m;
fa_w = 0;

% Simbolic state space matrix
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

% Transfer function
transfer_fct = C*inv(s*eye(4)-A)*B