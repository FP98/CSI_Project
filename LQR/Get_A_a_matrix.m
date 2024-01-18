%% Rename params
J = s_param.J;     
m = s_param.m;     
b = s_param.b;      
beta = s_param.beta;    
g = s_param.g;     
l = s_param.l;   

syms iz itheta z theta dz dtheta fm fa;

x_a = [iz itheta z, theta, dz, dtheta]; % Augmented state vector
f_a = [z; theta; dz; dtheta; -b/m * dz + fm/m * cos(theta) - g; -beta/J * dtheta + 2 * l/J * (fa)];

a = jacobian(f_a(1), x_a);
b = jacobian(f_a(2), x_a);
c = jacobian(f_a(3), x_a);
d = jacobian(f_a(4), x_a);
e = jacobian(f_a(5), x_a);
f = jacobian(f_a(6), x_a);

A_a = [a;b;c;d;e;f];

matlabFunction(A_a,'file','A_a_matrix', 'Vars', {x_a,fm, fa});
