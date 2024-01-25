% This script compute and plot Sensitivity and ComplSensitivity of the
% system y = T*r + S*T -T*n
S = inv(eye(2) + G*K);
T = eye(2) - S;
sigma(S);
hold on
grid on
sigma(T);
