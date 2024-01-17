%Def di sistema incerto

% Definisco paramentri incerti di sistema massa molla smorzatore
m = ureal('m', 3, 'Percentage', [-40, 40]); %valore nominale 3 che varia in un range di + o - 40%
c = ureal('m', 1, 'Percentage', [0.8, 1.2]);
k = ureal('m', 2, 'Percentage', [-0.6, 0.6]);

% Matrici del sistema
A = [0 1; -k/m -c/m];
B = [0 1/m]';
C = [1 0];
D = 0;

Sys = ss(A,B,C,D);
