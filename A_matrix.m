function Ak = A_matrix(in1,fm,fa)
%A_matrix
%    Ak = A_matrix(IN1,FM,FA)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    15-Jan-2024 18:19:23

theta = in1(:,2);
Ak = reshape([0.0,0.0,0.0,0.0,0.0,0.0,fm.*sin(theta).*(-5.0e-4),0.0,1.0,0.0,-3.0./4.0e+1,0.0,0.0,1.0,0.0,-3.0./1.0e+3],[4,4]);
end
