%% Weights of parametric uncertanity

%lm1
omega = logspace(-20,0,100);
bodemag((Gu(1,1)-Gu(1,1).NominalValue)/Gu(1,1).NominalValue);
hold on 
grid on

ord = 1;

[freq,resp_dB] = ginput(20);

resp = zeros(1,20);
for i = 1:20
    resp(i) = 10^(resp_dB(i)/20);
end

sys = frd(resp,freq);
W = fitmagfrd(sys,ord);

lm1 = tf(W*f);
bodemag(W,'r-',omega)
pause(2);
hold off

%lm2
omega = logspace(-20,-0,100);
bodemag((Gu(2,2)-Gu(2,2).NominalValue)/Gu(2,2).NominalValue);
hold on 
grid on

ord = 1;

[freq,resp_dB] = ginput(20);

resp = zeros(1,20);
for i = 1:20
    resp(i) = 10^(resp_dB(i)/20);
end

sys = frd(resp,freq);
W = fitmagfrd(sys,ord);

lm2 = tf(W*f);
bodemag(W,'r-',omega)
hold off