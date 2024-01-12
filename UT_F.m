function [p_mean, propagated_cov] = UT_F(x_mean, sigma_x, dt, fm, fa, s_param, Q, U_mean)

%% Rename params
J = s_param.J;     
m = s_param.m;     
b = s_param.b;      
beta = s_param.beta;    
g = s_param.g;     
l = s_param.l;  

% Define vector for UT
x = [x_mean; U_mean];
 
% Define Covariance for UT
sigma = blkdiag(sigma_x,Q);

% Define parameters
alpha = 1;
beta = 2;
k = 0;
n = size(x,1);
lambda = alpha^2*(n + k) - n;

% Compute weights
w0 = lambda/(lambda + n);
wc = zeros(2*n+1,1);
wc(1) = w0 + 1 - alpha^2 + beta;
wc(2:2*n+1) = 1/(2*(n+lambda));

wm = wc;
wm(1) = w0;

% Factorise covariance matrix whit SVD
[U,S] = svd(sigma);
S = complex(S);
gamma = U*S^(1/2);

% Factorise covariance matrix whit SQRT
% gamma = sqrt(complex(sigma));

gamma = real(gamma);

% Create sigma points
sigma_points = zeros(length(x),2*n+1);
sigma_points(:,1) = x;

for i = 1:size(gamma,2)
    sigma_points(:,i+1) = x + sqrt(n+lambda)*gamma(:,i);
    sigma_points(:,n+i+1) = x - sqrt(n+lambda)*gamma(:,i);
end

% Propagated sigma points eval.
prop_sigma_points = zeros(length(x_mean),2*n+1);
for i = 1:size(sigma_points,2)

    % Rename sigma points componets
    % z = sigma_points(1,i);
    theta = sigma_points(2,i);
    dz= sigma_points(3,i);
    dtheta = sigma_points(4,i);
    U(1) = sigma_points(5,i);
    U(2) = sigma_points(6,i);
    
    % F(sigma_points)
    prop_sigma_points(:,i) = sigma_points(1:4,i) + dt*...
        [dz;...
        dtheta;...
        (-b/m * dz + (fm + U(1))/m * cos(theta) -g);...
        (-beta/J * dtheta + 2 * l/J * (fa + U(2)))];
end

% Propagated mean
p_mean = zeros(length(x_mean), 1);
p_mean(1) = prop_sigma_points(1,:)*wm;
p_mean(2) = atan2(sin(prop_sigma_points(2,:))*wm,cos(prop_sigma_points(2,:))*wm);  
p_mean(3) = prop_sigma_points(3,:)*wm;
p_mean(4) = prop_sigma_points(4,:)*wm;

% Compute tilde
p_tilde = zeros(length(x_mean),2*n+1);

for i = 1:1:2*n+1
    p_tilde(1,i) = prop_sigma_points(1,i) - p_mean(1);
    p_tilde(2,i) = atan2(sin(prop_sigma_points(2,i) - p_mean(2)) ,cos(prop_sigma_points(2,i)- p_mean(2)));
    p_tilde(3,i) = prop_sigma_points(3,i) - p_mean(3);
    p_tilde(4,i) = prop_sigma_points(4,i) - p_mean(4);
end


% Propagated cov
propagated_cov = zeros(size(sigma_x));
    for i = 1:size(sigma_points,2)
        propagated_cov = propagated_cov + wc(i)*p_tilde(:,i)*p_tilde(:,i)';
    end
end


