function draw_sys(x, z, theta, s_param, z_0, z_w)
% Function used to draw the system in a specific configuration 

des = ["Convertiplane motion"];
% Scaling wing span for graphi simulation
l = s_param.l*4;

hold on
title(des);
axis equal
grid on

% Graph boundary evaluation
if z < 1.3*z_w
    axis ([x - 4*l x + 4*l 0.7*z_0 1.3*z_w]);
else
    axis ([x - 4*l x + 4*l 0.7*z_0 1.3*z]);
end

% Draw strarting altitude
plot([x - 4*l , x + 4*l],[z_0,z_0], '-b', 'LineWidth',1)

% Draw target altitude
plot([x - 4*l , x + 4*l],[z_w,z_w], '-r', 'LineWidth',1)

% Draw convertiplane fuselage
plot(x, z, 'ko', 'LineWidth',4);


% Half wing position 
HW_end1 = [x + l*cos(theta), z + l*sin(theta)];
HW_end2 = [x - l*cos(theta), z - l*sin(theta)];

% Draw half wings
plot([x, HW_end1(1)], [z, HW_end1(2)], '-k', 'LineWidth',2);
plot([x, HW_end2(1)], [z, HW_end2(2)], '-k', 'LineWidth',2);

end

