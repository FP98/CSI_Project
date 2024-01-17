%Def di sistema incerto

% Definisco paramentri incerti di sistema massa molla smorzatore
m = ureal('m', 3, 'Percentage', [-40, 40]); %valore nominale 3 che varia in un range di + o - 40%
c = ureal('c', 1, 'Range', [0.8, 1.2]);
k = ureal('k', 2, 'PlusMinus', [-0.6, 0.6]);

% Matrici del sistema
A = [0 1; -k/m -c/m];
B = [0 1/m]';
C = [1 0];
D = 0;

Sys = ss(A,B,C,D);

Gs = tf(1, [m, c, k]);

%comandi utili

usubs(Sys, 'm', 3, 'c', 1, 'k', 2); % Sostituisce ad interno del mio sistema i valori di k m c che voglio io

usample(Sys, 10);        % Estraggo random 10 sistemi compatibili con glii intervalli di incertezza che ho definito

figure(1)

hold on
bodemag(Sys) % Plotta diagramma di bode con 20 campioni in cui fa variare i parametri incerti negli intervalli compatibili
grid on

figure(2)
hold on
step(Sys)

B = gridureal(Sys, 10); % Genera 10 estrazioni di impianti e campiona in maniera uniforme
figure(3)
hold on 
step(B)

figure(4)
hold on
bodemag(B)

ucomplexm