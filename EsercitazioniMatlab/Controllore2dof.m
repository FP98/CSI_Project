% Controllore a 2 dof

s = tf('s');
G = 200/(10*s + 1)/(0.05*s + 1)^2;
Gd = 100/(10*s + 1);

%funzione peso di prestazione
M = 1.5;
A = 1e-4;
wb = 10;
w1 = tf([1/M wb],[1 wb*A]); % pesa errore di tracking passo a. tf due vettori in cui al numeratore ho coefficienti del vettore più a sinistrea, e al denominatore coefficienti di vettore più a destra
w2 = tf(1);
% Connect per P-K
 % ingressi e uscite di G
G.u = 'u'; %ingresso di G il comando G.u significa l 'input name di G e l ingresso lo chiamo u(potrei chiamarlo anche in maniera diversa)
G.y = 'y1';

% ingressi e uscite di Gd
Gd.u = 'd';
Gd.y = 'd1';

%ingressi ed uscite di w1
w1.u = 'e';
w1.y = 'z1';

%ingressi ed uscite di w1
w2.u = 'u';
w2.y = 'z2';

% nodi sommatori
Sum2 = sumblk('y = y1 + d1');
Sum3 = sumblk('e = y - r');           % M0 = 1

%connect
P = connect(G, Gd, w1, w2, Sum2, Sum3, {'r', 'd', 'u'}, {'z1', 'z2', 'r', 'y'}); % ottengo P in forma state space

Pzpk = zpk(P);                           % in questo modo ottengo matrice di trasferimento scritta come la vedo nelle slide

% Hinf controller
[K,CL, gamma] = hinfsyn(P, 2, 1);        % la CL sarà la LFT, gamma mi dirà quanto mi allontano da norma Hinf. il comando hinfsyn mi da valori in forma di state space

zpk(K);                                   % mi da controllore con funzioni di trasferimento

Kzpk = minreal(zpk(K), 1e-4);              % k in forma di matrice di trasferimento e facendo pulizia per valori inferiori ad e-4

% Ky e Kr controllers
Ky = -Kzpk(2);                             %seconda componente di K cambiata di segno
Kr = minreal(-Kzpk(1)/Kzpk(2), 1e-4);

% Step
T = Kr * feedback(Ky*G, 1);        % fdt fra riferimento ed uscita
Td = Gd/(1 + Ky*G);                % fdt tra disturbo ed uscita
Tur = Kr*Ky/(1+ Ky*G);             % fdt fra ingresso u ed r (sforzo di controllo per portare uscita al riferimento desiderato)
Tud = Ky*Gd/(1+ Ky*G);

figure(1)
hold on
step(T)
grid on

figure(2)
hold on
step(Td)
grid on

figure(3)
hold on
step(Tur)
grid on

figure(4)
hold on
step(Tud)
grid on