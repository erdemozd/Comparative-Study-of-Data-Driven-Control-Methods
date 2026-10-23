function [u0,info] = mpc_control( ...
    xk, ...
    rPreview, ...
    rDotPreview, ...
    p, ...
    ctrl)
% MPC_CONTROL
%
% q3-only position tracking with internally optimized steady input.

%% 1. Inputs

xk = xk(:);

rPreview = rPreview(:);
rDotPreview = rDotPreview(:); %#ok<NASGU>

p = p; %#ok<NASGU>

N = ctrl.N;

rTarget = ...
    rPreview(1);

%% 2. Desired physical equilibrium
%
% This specifies geometry only.
%
% The equilibrium INPUT is not supplied.

xTarget = [ ...
    rTarget;
    0;
    rTarget;
    0;
    rTarget;
    0];

Xreference = ...
    kron( ...
    ones(N,1), ...
    xTarget);

%% 3. Affine prediction offset

xOffset = ...
    ctrl.Sx*xk;

%% 4. Objective linear term

stateErrorOffset = ...
    xOffset ...
    - ...
    Xreference;

f = ...
    2*ctrl.Ax' ...
    *ctrl.Qbar ...
    *stateErrorOffset;

%% 5. Terminal state equality
%
% Tx*(xOffset + Su*U)
%
% =
%
% 1 kron xTarget

AeqTerminalX = [ ...
    ctrl.Tx*ctrl.Su, ...
    zeros(6*ctrl.nTerm,1)];

beqTerminalX = ...
    ctrl.ExTerm*xTarget ...
    - ...
    ctrl.Tx*xOffset;

%% Terminal input equilibrium
%
% Tu*U = 1*u_s

Aeq = [ ...
    AeqTerminalX;
    ctrl.AeqTerminalU];

beq = [ ...
    beqTerminalX;
    zeros(ctrl.nTerm,1)];

%% 6. Future-position constraints

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
    Aeq, ...
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

    info.uPred = [];
    info.xPred = [];

    info.uSteady = NaN;

    info.output = output;

    return;

end

%% 9. Extract

U = ...
    z(ctrl.uIndex);

uSteady = ...
    z(ctrl.usIndex);

xPredVector = ...
    xOffset ...
    + ...
    ctrl.Su*U;

xPred = ...
    reshape( ...
    xPredVector, ...
    6, ...
    N);

u0 = ...
    U(1);

%% 10. Return

info.success = true;
info.exitflag = exitflag;
info.solveTime = solveTime;

info.J = J;

info.uPred = U;
info.xPred = xPred;

info.uSteady = uSteady;

info.output = output;

end