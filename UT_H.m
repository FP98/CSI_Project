function [y, Cov_y, cross_cov] = UT_H(x_mean, sigma_x,C)


% Define parameters
alpha = 1;
beta = 2;
k = 0;
n = size(x_mean,1);
lambda = alpha^2*(n + k) - n;



% Compute weights
w0 = lambda/(lambda + n);

wc = zeros(2*n+1,1);
wc(1) = w0 + 1 - alpha^2 + beta;
wc(2:2*n+1) = 1/(2*(n+lambda));


% Mean weights
wm = wc;
wm(1) = w0;

% Factorise covariance matrix whit SVD
[U,S] = svd(sigma_x);
S = complex(S);                 % convert S to a complex matrix to avoid errors
gamma = U*sqrt(S);


% Factorise covariance matrix whit SQRT
% gamma = sqrt(complex(sigma_x));

gamma = real(gamma);

% Create sigma points
sigma_points = zeros(length(x_mean),2*n+1);
sigma_points(:,1) = x_mean;

for i = 1:size(gamma,2)
    sigma_points(:,i+1) = x_mean + sqrt(n+lambda)*gamma(:,i);
    sigma_points(:,n+i+1) = x_mean - sqrt(n+lambda)*gamma(:,i);
end

% Propagated sigma points eval.
prop_sigma_points = zeros(2,2*n+1);
for i = 1:size(sigma_points,2)

    % Rename sigma points componets
    z = sigma_points(1,i);
    theta = sigma_points(2,i);
    dz = sigma_points(3,i);
    dtheta = sigma_points(4,i);
    
    % h(sigma_points)
    prop_sigma_points(:,i) = C*[z; theta; dz; dtheta];
end


% Compute means
y = zeros(size(prop_sigma_points,1),1);
y(1) = prop_sigma_points(1,:)*wm;
y(2) = atan2(sin(prop_sigma_points(2,:))*wm,cos(prop_sigma_points(2,:))*wm);


% Compute tilde
y_tilde = zeros(size(prop_sigma_points,1),2*n+1);

for i = 1:1:2*n+1
    y_tilde(1,i) = prop_sigma_points(1,i) - y(1);
    y_tilde(2,i) = atan2(sin(prop_sigma_points(2,i) - y(2)) ,cos(prop_sigma_points(2,i)- y(2)));
end

% Compute cov
Cov_y = zeros(length(y), length(y));
cross_cov = zeros(length(x_mean), length(y));
for i = 1:1:2*n+1
        Cov_y = Cov_y + wc(i)*y_tilde(:,i)*y_tilde(:,i)';
        cross_cov = cross_cov + wc(i)*(sigma_points(:,i) - x_mean)*y_tilde(:,i)';
end

end