%% Rename params
J = s_param.J;     
m = s_param.m;     
b = s_param.b;      
beta = s_param.beta;    
g = s_param.g;     
l = s_param.l;   

syms iz itheta z theta dz dtheta fm fa;

u = [fm, fa];
f_a = [z; theta; dz; dtheta; -b/m * dz + fm/m * cos(theta) - g; -beta/J * dtheta + 2 * l/J * (fa)];
a = jacobian(f_a(1), u);
b = jacobian(f_a(2), u);
c = jacobian(f_a(3), u);
d = jacobian(f_a(4), u);
e = jacobian(f_a(5), u);
f = jacobian(f_a(6), u);
B_a = [a;b;c;d;e;f];

matlabFunction(B_a,'file','B_a_matrix', 'Vars', {x_a,fm,fa});