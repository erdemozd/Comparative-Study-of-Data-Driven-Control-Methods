function [u0,info] = deepc_control_terminal_frobenius( ...
    uIni,yIni,rPreview,rDotPreview,p,ctrl)

uIni = uIni(:);
yIni = yIni(:);

rPreview = rPreview(:);
rDotPreview = rDotPreview(:); %#ok<NASGU>
p = p; %#ok<NASGU>

rTarget = rPreview(1);

xTarget = [
    rTarget;
    0;
    rTarget;
    0;
    rTarget;
    0];

Xreference = kron(ones(ctrl.N,1),xTarget);
terminalTarget = ctrl.ExTerm*xTarget;

fRunning = -2*ctrl.Ax'*ctrl.Qbar*Xreference;

fTerminal = -2*ctrl.lambdaTerminalX ...
    *ctrl.AtermX' ...
    *ctrl.WtermX ...
    *terminalTarget;

f = fRunning + fTerminal;

beq = [
    uIni;
    yIni];

tic;

[z,J,exitflag,output] = quadprog( ...
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

solveTime = toc;

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

    info.terminalStateNorm = NaN;
    info.terminalInputNorm = NaN;

    info.output = output;

    return;
end

g = z(ctrl.gIndex);
sigma = z(ctrl.sigmaIndex);
uSteady = z(ctrl.usIndex);

uPred = ctrl.Uf*g;

xPredVector = ctrl.Yf*g;
xPred = reshape(xPredVector,ctrl.ny,ctrl.N);

u0 = uPred(1);

pastInputResidual = norm(ctrl.Up*g-uIni,2);
pastStateResidual = norm(ctrl.Yp*g-sigma-yIni,2);

terminalStateVector = ctrl.Tx*xPredVector-terminalTarget;
terminalStateMatrix = reshape(terminalStateVector,ctrl.ny,ctrl.nTerm);

terminalInputVector = ctrl.Tu*uPred-uSteady*ones(ctrl.nTerm,1);

weightedTerminalState = sqrt(ctrl.Wy)*terminalStateMatrix;

terminalStateNorm = norm(weightedTerminalState,'fro');
terminalInputNorm = norm(terminalInputVector,'fro');

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

info.terminalStateNorm = terminalStateNorm;
info.terminalInputNorm = terminalInputNorm;

info.output = output;

end