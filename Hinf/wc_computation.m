function P_wc = wc_computation(wcu_perf,W1,W2)
s = tf('s');
% Rename input
J = wcu_perf.J;    
m = wcu_perf.m;
b = wcu_perf.b;
beta = wcu_perf.beta;
l = wcu_perf.l;
Km1 = wcu_perf.Km1;
Km2 = wcu_perf.Km2;
T1 = wcu_perf.T1;
T2 = wcu_perf.T2;
Tm1 = wcu_perf.Tm1;
Tm2 = wcu_perf.Tm2;

% Worst case transfer matrix
G1_wc = [1/(s*(b + m*s)) 0; 0 (2*l)/(s*(beta + J*s))];
Gm1_wc = Km1*(1/(T1/2*s+1))*(1/(Tm1*s+1));
Gm2_wc = Km2*(1/(T2/2*s+1))*(1/(Tm2*s+1));
G_wc = G1_wc*blkdiag(Gm1_wc,Gm2_wc);

% Connect P-K

% Input and output of G_wc
G_wc.u = 'u1';
G_wc.y = 'y1';

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
P_wc = connect(G_wc, W1, W2, S1, S2, S3, {'r', 'd', 'n', 'u'}, {'z1', 'z2', 'e'});
end