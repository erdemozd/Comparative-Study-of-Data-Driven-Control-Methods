function s = benchmark_settings(p)
% BENCHMARK_SETTINGS Common initial benchmark settings.

s.Tini = 10;
s.N = 40;
s.Tend = 30.0;

% q3 is the main tracked position.
% q1/q2 positions are left free but constrained.
s.Q = diag([10, 8, 10, 12, 180, 25]);
s.R = 0.10;

s.lambdaG = 1e-2;
s.lambdaSigma = 1e5;

s.uMax = p.uMax;
s.qMax = p.qMax;

% Default comparison uses absolute input magnitude, not an analytical
% equilibrium-input reference, so standard DeePC receives no model formula.
s.useEquilibriumInputReference = false;

end
