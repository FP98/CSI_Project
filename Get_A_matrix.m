%% Rename params
J = s_param.J;     
m = s_param.m;     
b = s_param.b;      
beta = s_param.beta;    
g = s_param.g;     
l = s_param.l;   

syms z theta dz dtheta fm fa;

x = [z, theta, dz, dtheta];
f = [dz; dtheta; -b/m * dz + fm/m * cos(theta) - g; -beta/J * dtheta + 2 * l/J * (fa)];

a = jacobian(f(1), x);
b = jacobian(f(2), x);
c = jacobian(f(3), x);
d = jacobian(f(4), x);

Ak = [a;b;c;d];

matlabFunction(Ak,'file','A_matrix', 'Vars', {x,fm, fa});
