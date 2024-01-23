% MU-analysis

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
% Gm1 = Km1*(1/(T1/2*s+1))*(1/(Tm1*s+1));
% Gm2 = Km2*(1/(T2/2*s+1))*(1/(Tm2*s+1));

% G_u(s) global
G = G1*blkdiag(Gm1,Gm2);

%% Connect Uncertain P-K

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
%% N & Delta definition
F = lft(P,K);
[N,Delta,BlkStruct] = lftdata(F);

%% M definition
szDelta = size(Delta);
M = N(1:szDelta(2),1:szDelta(1));
omega = logspace(-3, 3, 61);
Mfr = frd(M, omega);
%% mussv
[mubnds, muinfo] = mussv(Mfr, BlkStruct);
muRP = mubnds(:,1);
[muRPinf,muRPw] = norm(muRP,inf);

bodemag(muRP); 
grid on

%% Rob stab & Rob gain
usys= lft(Delta, N);
[stabmarg,wcu_stab] = robstab(usys);
[perfmarg,wcu_perf] = robgain(usys,gamma);

wcsys = usubs(usys, wcu_perf);
