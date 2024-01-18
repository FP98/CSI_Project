%% Hinf controller

% Nominal plant
s = tf('s');
lin_sys = ss(A,B,C,D);
G1 = tf(lin_sys);

% Delay approximation
delay1= tf(exp(-T1*s));
delay2= tf(exp(-T2*s));
sys1 = pade(delay1,2);
sys2 = pade(delay2,2);

% Input transfer function
Gm1 = sys1*tf([Km1],[Tm1 1]);    
Gm2 = sys2*tf([Km2],[Tm2 1]);    

% Global plant (input + plant)
G = G1*blkdiag(Gm1,Gm2);

Gd = [G1 eye(2)];

% Weights definitions
M = 1.5;
A_ = 10;
wb = 10; 

% High pass filters
w1_11 = tf([1/M wb],[1 wb*A_]);
w1_22 = tf([1/M wb],[1 wb*A_]);

% Weight matrix
W1 = blkdiag(w1_11,w1_22);
W2 = blkdiag(tf(1), tf(1));

%% Connect P-K

% Input and output of G
G.u = 'u';
G.y = 'y1';

% Input and output of Gd
Gd.u = 'd';
Gd.y = 'd1';

% Input and output of W1
W1.u = 'e';
W1.y = 'z1';

% Input and output of W2
W2.u = 'u';
W2.y = 'z2';

% Sum nodes
S2 = sumblk('y = y1 + d1',2);
S3 = sumblk('e = y - r',2);       % M0 = I

% Connect
P = connect(G, Gd, W1, W2, S2, S3, {'r', 'd', 'u'}, {'z1', 'z2', 'r', 'y'});
Pzpk = zpk(P);

P1 = augw(G,W1,W2,[]);

% Hinf
[K,CL, gamma] = hinfsyn(P1, 4, 2);



