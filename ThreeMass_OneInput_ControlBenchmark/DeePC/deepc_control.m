function [u0,info] = deepc_control( ...
    uIni, ...
    yIni, ...
    rPreview, ...
    rDotPreview, ...
    p, ...
    ctrl)
% DEEPC_CONTROL
%
% Standard DeePC with optimized steady input u_s.
%
% q3 is the only position tracked in the running cost.
%
% The full coordinated configuration is imposed only at the terminal
% equilibrium.

%% 1. Inputs

uIni = uIni(:);
yIni = yIni(:);

rPreview = rPreview(:);
rDotPreview = rDotPreview(:); %#ok<NASGU>

p = p; %#ok<NASGU>

rTarget = ...
    rPreview(1);

%% 2. Reference trajectory
%
% q1 and q2 appear here, but their Q weights are zero.
%
% Thus only q3 position is actually tracked during the horizon.

xTarget = [ ...
    rTarget;
    0;
    rTarget;
    0;
    rTarget;
    0];

Xreference = ...
    kron( ...
    ones(ctrl.N,1), ...
    xTarget);

%% 3. Objective linear term

f = ...
    -2*ctrl.Ax' ...
    *ctrl.Qbar ...
    *Xreference;

%% 4. Equality RHS

terminalTarget = ...
    ctrl.ExTerm*xTarget;

beq = [ ...
    uIni;
    yIni;
    terminalTarget;
    zeros(ctrl.nTerm,1)];

%% 5. Solve

tic;

[z,J,exitflag,output] = ...
    quadprog( ...
    ctrl.H, ...
    f, ...
    ctrl.Aineq, ...
    ctrl.bineq, ...
    ctrl.Aeq, ...
    beq, ...
    [], ...
    [], ...
    [], ...
    ctrl.opts);

solveTime = ...
    toc;

%% 6. Failure

if exitflag <= 0 || isempty(z)

    u0 = NaN;

    info.success = false;
    info.exitflag = exitflag;
    info.solveTime = solveTime;

    info.J = NaN;

    info.g = [];
    info.sigmaY = [];
    info.sigmaNorm = NaN;

    info.uPred = [];
    info.xPred = [];

    info.uSteady = NaN;

    info.output = output;

    return;

end

%% 7. Extract

g = ...
    z(ctrl.gIndex);

sigma = ...
    z(ctrl.sigmaIndex);

uSteady = ...
    z(ctrl.usIndex);

%% 8. Predictions

uPred = ...
    ctrl.Uf*g;

xPredVector = ...
    ctrl.Yf*g;

xPred = ...
    reshape( ...
    xPredVector, ...
    ctrl.ny, ...
    ctrl.N);

u0 = ...
    uPred(1);

%% 9. Diagnostics

pastInputResidual = ...
    norm( ...
    ctrl.Up*g-uIni, ...
    2);

pastStateResidual = ...
    norm( ...
    ctrl.Yp*g ...
    - ...
    sigma ...
    - ...
    yIni, ...
    2);

%% 10. Return

info.success = true;
info.exitflag = exitflag;
info.solveTime = solveTime;

info.J = J;

info.g = g;

info.sigmaY = sigma;
info.sigmaNorm = norm(sigma,2);

info.pastInputResidual = pastInputResidual;
info.pastStateResidual = pastStateResidual;

info.uPred = uPred;
info.xPred = xPred;

info.uSteady = uSteady;

info.output = output;

end