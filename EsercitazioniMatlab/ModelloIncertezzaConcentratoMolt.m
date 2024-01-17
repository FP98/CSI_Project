% Modello di incertezza concentrato
% Modello Moltiplicativo

omega = logspace(-1, 3, 100);

K0 = 1; T0 = 1/15;

Gnom = tf( [K0], [T0 1]);
Gnom_frd = frd(Gnom, omega); %valuto Gnom in omega

figure(1)
hold off

for K = 0.8*K0:0.08*K0:1.2*K0
    for T = 0.7*T0:0.06*T0:1.3*T0
        G = tf([K], [T 1]);
        G_frd = frd(G, omega);
        reldiff = (G_frd - Gnom_frd)/Gnom_frd;
        bodemag(reldiff, 'c--', omega)
        hold on 
    end 
end
grid on

% Calcolo del peso

ord = 1; %scelgo peso di ordine due per iniziare

[freq, resp_db] = ginput(20);

%resp = zeros(1, 20);%
for i = 1:20
    resp(i) = 10^(resp_db(i)/20);
end

sys = frd(resp, freq);
W = fitmagfrd(sys, ord);

Wm = tf(W); %peso modello incertezza additiva

bodemag(Wm, 'r-', omega)

%Modello concentrato (lumped) additivo

Delta_m = ultidyn('Delta_m', [1 1]);

Gp = Gnom*(1 + Wm*Delta_m); % Questo sarà il nostro modello