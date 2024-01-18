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

%% Weights definitions
M = 1.5;
A_ =1e-4;
wb = 10; 

% High pass filters
w1_11 = tf([1/M wb],[1 wb*A_]);
w1_22 = tf([1/M wb],[1 wb*A_]);

% Weight matrix
W1 = blkdiag(w1_11,w1_22);
W2 = blkdiag(tf(0.01), tf(0.01));


% Hinf
[K,CL, gamma] = mixsyn(G, W1, W2, []);
K=tf(ss(K.A,K.B,K.C,K.D));


