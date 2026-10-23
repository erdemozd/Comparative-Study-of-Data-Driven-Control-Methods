function [u0,info] = sdmpc_control( ...
    uIni, ...
    resIni, ...
    xM0, ...
    rPreview, ...
    rDotPreview, ...
    p, ...
    ctrl)
% SDMPC_CONTROL
%
% q3-only tracking with internally optimized steady input.

%% 1. Inputs

uIni = uIni(:);
resIni = resIni(:);
xM0 = xM0(:);

rPreview = rPreview(:);
rDotPreview = rDotPreview(:); %#ok<NASGU>

p = p; %#ok<NASGU>

rTarget = ...
    rPreview(1);

%% 2. Equilibrium geometry

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

%% 3. Prediction offset

xOffset = ...
    ctrl.Sx*xM0;

%% 4. Objective

stateErrorOffset = ...
    xOffset ...
    - ...
    Xreference;

f = ...
    2*ctrl.Ax' ...
    *ctrl.Qbar ...
    *stateErrorOffset;

%% 5. Equality RHS

terminalTarget = ...
    ctrl.ExTerm*xTarget;

terminalOffset = ...
    ctrl.Tx*xOffset;

beq = [ ...
    uIni;
    resIni;
    terminalTarget-terminalOffset;
    zeros(ctrl.nTerm,1)];

%% 6. Position constraints

positionOffset = ...
    ctrl.SqBar*xOffset;

nPos = ...
    numel(positionOffset);

bposition = [ ...
    ctrl.qMax*ones(nPos,1) ...
    - ...
    positionOffset;
    ...
    ctrl.qMax*ones(nPos,1) ...
    + ...
    positionOffset];

bineq = [ ...
    ctrl.binput;
    bposition;
    ctrl.bsteadyInput];

%% 7. Solve

tic;

[z,J,exitflag,output] = ...
    quadprog( ...
    ctrl.H, ...
    f, ...
    ctrl.Aineq, ...
    bineq, ...
    ctrl.Aeq, ...
    beq, ...
    [], ...
    [], ...
    [], ...
    ctrl.opts);

solveTime = ...
    toc;

%% 8. Failure

if exitflag <= 0 || isempty(z)

    u0 = NaN;

    info.success = false;
    info.exitflag = exitflag;
    info.solveTime = solveTime;

    info.J = NaN;

    info.g = [];
    info.sigmaNorm = NaN;

    info.uPred = [];
    info.xPred = [];

    info.uSteady = NaN;

    info.output = output;

    return;

end

%% 9. Extract

g = ...
    z(ctrl.gIndex);

sigma = ...
    z(ctrl.sigmaIndex);

uSteady = ...
    z(ctrl.usIndex);

uPred = ...
    ctrl.Uf*g;

xPredVector = ...
    xOffset ...
    + ...
    ctrl.G*g;

xPred = ...
    reshape( ...
    xPredVector, ...
    6, ...
    ctrl.N);

u0 = ...
    uPred(1);

%% 10. Return

info.success = true;
info.exitflag = exitflag;
info.solveTime = solveTime;

info.J = J;

info.g = g;
info.sigmaNorm = norm(sigma,2);

info.uPred = uPred;
info.xPred = xPred;

info.uSteady = uSteady;

info.output = output;

end