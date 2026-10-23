function dx = three_mass_nonlinear(~,x,u,p)
% THREE_MASS_NONLINEAR Ground-truth continuous-time nonlinear plant.
%
% x = [q1; v1; q2; v2; q3; v3]
% u = force on mass 1

q1 = x(1); v1 = x(2);
q2 = x(3); v2 = x(4);
q3 = x(5); v3 = x(6);

dq12 = q2-q1;
dv12 = v2-v1;

dq23 = q3-q2;
dv23 = v3-v2;

F12 = p.k12*dq12 + p.c12*dv12;

F23 = ...
    p.k23*dq23 ...
    + p.k3*dq23^3 ...
    + p.c23*dv23;

a1 = ...
    (u ...
    - p.k1*q1 ...
    - p.c1*v1 ...
    + F12) / p.m1;

a2 = ...
    (-F12 + F23) / p.m2;

a3 = ...
    (-F23) / p.m3;

dx = [ ...
    v1;
    a1;
    v2;
    a2;
    v3;
    a3];

end
