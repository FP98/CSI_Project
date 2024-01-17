s=tf('s');
G=1/(0.2*s+1)/(s+1)*[1 1;1+2*s 2];

%Poli e Zeri
Poles_MIMO=pole(G);
Zeros_MIMO=tzero(G);

%Risposta al gradino
step(G*[-0.5 0.5]') %moltiplico per il vettore per "ingannare" comando step 
grid on;
%Design 1
wu = 1;
Wu  = blkdiag(wu,wu);
A = 1e-4;
M = 1.5;
wB1 = Zeros_MIMO/2;
wB2 = wB1;
wP1 = (s/M+wB1)/(s+wB1*A);
wP2 = (s/M+wB2)/(s+wB2*A);
WP = blkdiag(wP1,wP2); %matrice peso per S
[K1, CLaug1,GAM1, ~]= mixsyn(G,WP,Wu,[]);

%plot dei valori singolari
sigmaplot(inv(WP),inv(eye(2) + G*K1))
grid on

%step in anello chiuso
step(G*K1*inv(eye(2) + G*K1)*[1 -1]')
grid on
hold off
