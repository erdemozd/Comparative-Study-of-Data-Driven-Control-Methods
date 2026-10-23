function [A,B,C,D] = three_mass_nominal_linear(p)
% THREE_MASS_NOMINAL_LINEAR
%
% Nominal/local linear model obtained by omitting
%     k3*(q3-q2)^3.
%
% This is also the exact Jacobian linearization at q3-q2 = 0.

A = [ ...
    0, 1, 0, 0, 0, 0;
    -(p.k1+p.k12)/p.m1, -(p.c1+p.c12)/p.m1, ...
      p.k12/p.m1,        p.c12/p.m1, 0, 0;
    0, 0, 0, 1, 0, 0;
      p.k12/p.m2,        p.c12/p.m2, ...
    -(p.k12+p.k23)/p.m2, -(p.c12+p.c23)/p.m2, ...
      p.k23/p.m2,         p.c23/p.m2;
    0, 0, 0, 0, 0, 1;
    0, 0, ...
      p.k23/p.m3,         p.c23/p.m3, ...
    - p.k23/p.m3,       - p.c23/p.m3];

B = [ ...
    0;
    1/p.m1;
    0;
    0;
    0;
    0];

C = eye(6);
D = zeros(6,1);

end
