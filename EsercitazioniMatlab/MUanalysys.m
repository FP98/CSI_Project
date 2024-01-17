%% MU-analysys
G0 = [87.8 -86.4; 108.2 -109.6];
G = tf([1], [75 1])*G0; %impianto
G = minreal(ss(G));

% Weights
Wp = 0.5*tf([10 1], [10 1e-5])*eye(2);
Wi = tf([1 0.2], [0.5 1])*eye(2);

% Generalized Plant P
G.u = 'uG';
G.y = 'yG';

Wp.u = 'z1';
Wp.y = 'z';

Wi.u = 'u';
Wi.y = 'ydi';

Sum1 = sumblk('z1=w+yG', 2);
Sum2 = sumblk('uG=u+udi', 2);
Sum3 = sumblk('v=-z1', 2);      % Importante specificare la dimensione qui

P = connect(G, Wp, Wi, Sum1, Sum2, Sum3, ...
    {'udi', 'w', 'u'}, {'ydi', 'z', 'v'});     

Kinv = 0.7*tf([75 1], [1 1e-5])*(inv(G0));

N = lft(P, Kinv); % lower LFT N-Delta
omega = logspace(-3, 3, 61);
Nfr = frd(N, omega);

%mu per RP
blk = [1 1;1 1; 2 2];
[mubnds, muinfo] = mussv(Nfr, blk, 'c');      % Ci calcola il valore singolare strutturato
muRP = mubnds(:,1);
[muRPinf,muRPw] = norm(muRP,inf);

bodemag(muRP) % Plotto a tratto continuo

% rbstab (Robusta stabilità)
Delta = [ultidyn('Delta_1', [1,1]) 0; 0 ultidyn('Delta_2', [1,1])]; % delta generico con norma h infinito minore o uguale di 1

usys= lft(Delta, N);

[stabmarg,wcu] = robstab(usys);

wcusys = lft([wcu.Delta_1 0; 0 wcu.Delta_1], N); % worsth case sys

pole(wcusys) % calcolo i poli del sistema nel worst case

gamma = 6; % Non sa bene nemmeno lui cosa rappresenta
[perfmarg, wcu] = robgain(usys, gamma)