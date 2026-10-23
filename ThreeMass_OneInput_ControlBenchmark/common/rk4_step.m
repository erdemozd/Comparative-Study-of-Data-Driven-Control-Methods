function xNext = rk4_step(x,u,Ts,p)
% RK4_STEP Fixed-step RK4 under zero-order-held input.

f = @(xx) three_mass_nonlinear(0,xx,u,p);

k1 = f(x);
k2 = f(x + 0.5*Ts*k1);
k3 = f(x + 0.5*Ts*k2);
k4 = f(x + Ts*k3);

xNext = x + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);

end
