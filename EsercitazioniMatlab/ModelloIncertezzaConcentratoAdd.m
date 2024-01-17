% Modello di incertezza concentrato

% Modello additivo

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
        diff = G_frd - Gnom_frd;
        bodemag(diff, 'c--', omega)
        hold on 
    end 
end
grid on

% Calcolo del peso

ord = 2; %scelgo peso di ordine due per iniziare

[freq, resp_db] = ginput(20);

resp = zeros(1, 20);
for i = 1:20
    resp(i) = 10^(resp_db(i)/20);
end

sys = frd(resp, freq);
W = fitmagfrd(sys, ord);

Wa = tf(W); %peso modello incertezza additiva

bodemag(Wa, 'r-', omega)

%Modello concentrato (lumped) additivo

Delta_a = ultidyn('Delta_a', [1 1]);

Gp = Gnom + Wa*Delta_a; % Questo sarà il nostro modello

