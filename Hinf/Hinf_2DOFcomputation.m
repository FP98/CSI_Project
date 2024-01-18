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
Gd = [blkdiag(tf(1),tf(1)) blkdiag(tf(1),tf(1))];
%% Weights definitions
M = 1.5;
A_ =1e-4;
wb = 10; 

% High pass filters
w1_11 = tf([1/M wb],[1 wb*A_]);
w1_22 = tf([1/M wb],[1 wb*A_]);

% Weight matrix
W1 = blkdiag(w1_11,w1_22);
W2 = blkdiag(tf(10), tf(10));
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
S1 = sumblk('e = r - y',2);
S2 = sumblk('y = y1 + d1',2);       % M0 = I

% Connect
P = connect(G, Gd, W1, W2, S1, S2, {'r', 'd', 'u'}, {'z1', 'z2', 'e'});

% Hinf
[K_ms,CL, gamma] = mixsyn(G, W1, W2, []);
K_ms=tf(ss(K_ms.A,K_ms.B,K_ms.C,K_ms.D));

opts = hinfsynOptions('Method','RIC');
[K,CL, gamma] = hinfsyn(P, 2, 2, opts);
K = tf(ss(K.A,K.B,K.C,K.D));
