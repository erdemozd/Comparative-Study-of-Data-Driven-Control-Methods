function p = three_mass_params()
% THREE_MASS_PARAMS
%
% Three-mass benchmark:
%
% wall -- (k1,c1) -- m1 -- (k12,c12) -- m2 -- (k23,k3,c23) -- m3
%
% Nonlinear spring is only between masses 2 and 3.

p.m1 = 1.0;      % kg
p.m2 = 1.5;      % kg
p.m3 = 1.2;      % kg

p.k1  = 20.0;    % N/m, wall spring
p.k12 = 15.0;    % N/m, m1-m2 linear spring
p.k23 = 12.0;    % N/m, m2-m3 linear spring
p.k3  = 120.0;    % N/m^3, cubic stiffness between m2-m3

p.c1  = 1.0;     % N s/m
p.c12 = 0.8;     % N s/m
p.c23 = 0.7;     % N s/m

p.Ts = 0.05;     % s

p.uMax = 25.0;   % N
p.qMax = 0.9;   % m

end
