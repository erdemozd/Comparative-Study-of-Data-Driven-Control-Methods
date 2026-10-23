function [u0,info] = deepc_control_no_terminal( ...
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

f = -2*ctrl.Ax'*ctrl.Qbar*Xreference;

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