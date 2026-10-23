function [u0,info] = hdeepc_control( ...
    uIni, ...
    yuIni, ...
    xK0, ...
    rPreview, ...
    rDotPreview, ...
    p, ...
    ctrl)
% HDEEPC_CONTROL
%
% q3-only tracking + internally optimized steady input.

%% 1. Inputs

uIni = uIni(:);
yuIni = yuIni(:);
xK0 = xK0(:);

rPreview = rPreview(:);
rDotPreview = rDotPreview(:); %#ok<NASGU>

p = p; %#ok<NASGU>

rTarget = ...
    rPreview(1);

%% 2. Desired equilibrium geometry

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

%% 3. Affine prediction offset

xOffset = ...
    ctrl.Sfull*xK0;

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
    yuIni;
    terminalTarget-terminalOffset;
    zeros(ctrl.nTerm,1)];

%% 6. Position bounds

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
    ctrl.Gfull*g;

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