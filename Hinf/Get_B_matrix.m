%% Rename params
J = s_param.J;     
m = s_param.m;     
b = s_param.b;      
beta = s_param.beta;    
g = s_param.g;     
l = s_param.l;   

syms z theta dz dtheta fm fa;

u = [fm, fa];
f = [dz; dtheta; -b/m * dz + fm/m * cos(theta) - g; -beta/J * dtheta + 2 * l/J * (fa)];
a = jacobian(f(1), u);
b = jacobian(f(2), u);
c = jacobian(f(3), u);
d = jacobian(f(4), u);

B = [a;b;c;d];

matlabFunction(B,'file','B_matrix', 'Vars', {x,fm,fa});