function ctrl = deepc_setup_terminal_frobenius(p,deepcData)

s = benchmark_settings(p);

nu = 1;
ny = 6;

Tini = deepcData.Tini;
N = deepcData.N;
L = Tini+N;

nTerm = min(2,N);

lambdaTerminalX = 1.0;
lambdaTerminalU = 1.0;

Hu = block_hankel(deepcData.u,L);
Hy = block_hankel(deepcData.y,L);

Up = Hu(1:nu*Tini,:);
Uf = Hu(nu*Tini+1:end,:);

Yp = Hy(1:ny*Tini,:);
Yf = Hy(ny*Tini+1:end,:);

ng = size(Hu,2);

nSigma = ny*Tini;
nz = ng+nSigma+1;

gIndex = 1:ng;
sigmaIndex = ng+(1:nSigma);
usIndex = ng+nSigma+1;

Qbar = kron(eye(N),s.Q);
Rbar = kron(eye(N),s.R);

Eu = ones(N,1);

Ax = [Yf, zeros(ny*N,nSigma), zeros(ny*N,1)];
Au = [Uf, zeros(N,nSigma), -Eu];

Hreg = zeros(nz,nz);

Hreg(gIndex,gIndex) = 2*s.lambdaG*eye(ng);
Hreg(sigmaIndex,sigmaIndex) = 2*s.lambdaSigma*eye(nSigma);

Tx = [
    zeros(ny*nTerm,ny*(N-nTerm)), ...
    eye(ny*nTerm)];

Tu = [
    zeros(nTerm,N-nTerm), ...
    eye(nTerm)];

ExTerm = kron(ones(nTerm,1),eye(ny));
EuTerm = ones(nTerm,1);

AtermX = [
    Tx*Yf, ...
    zeros(ny*nTerm,nSigma), ...
    zeros(ny*nTerm,1)];

AtermU = [
    Tu*Uf, ...
    zeros(nTerm,nSigma), ...
    -EuTerm];

if isfield(deepcData,'sigmaY')
    sigmaY = deepcData.sigmaY(:);
else
    sigmaY = 0.05*sqrt(mean(deepcData.y.^2,2));
end

signalScale = sqrt(mean(deepcData.y.^2,2));
sigmaFloor = 0.01*max(signalScale,1e-6);

sigmaEffective = max(sigmaY,sigmaFloor);

Wy = diag(1./sigmaEffective.^2);
WtermX = kron(eye(nTerm),Wy);

HterminalX = 2*lambdaTerminalX*(AtermX'*WtermX*AtermX);
HterminalU = 2*lambdaTerminalU*(AtermU'*AtermU);

H = 2*(Ax'*Qbar*Ax + Au'*Rbar*Au) ...
    + Hreg ...
    + HterminalX ...
    + HterminalU;

H = 0.5*(H+H');

AeqInput = [
    Up, ...
    zeros(nu*Tini,nSigma), ...
    zeros(nu*Tini,1)];

AeqOutput = [
    Yp, ...
    -eye(nSigma), ...
    zeros(nSigma,1)];

Aeq = [
    AeqInput;
    AeqOutput];

Sq = [
    1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];

SqBar = kron(eye(N),Sq);

Yposition = SqBar*Yf;

Ainput = [
     Uf, zeros(N,nSigma+1);
    -Uf, zeros(N,nSigma+1)];

binput = s.uMax*ones(2*N,1);

Aposition = [
     Yposition, zeros(3*N,nSigma+1);
    -Yposition, zeros(3*N,nSigma+1)];

bposition = s.qMax*ones(6*N,1);

AsteadyInput = [
    zeros(1,ng+nSigma),  1;
    zeros(1,ng+nSigma), -1];

bsteadyInput = s.uMax*ones(2,1);

Aineq = [
    Ainput;
    Aposition;
    AsteadyInput];

bineq = [
    binput;
    bposition;
    bsteadyInput];

ctrl.nu = nu;
ctrl.ny = ny;

ctrl.Tini = Tini;
ctrl.N = N;
ctrl.L = L;

ctrl.nTerm = nTerm;

ctrl.ng = ng;
ctrl.nSigma = nSigma;
ctrl.nz = nz;

ctrl.gIndex = gIndex;
ctrl.sigmaIndex = sigmaIndex;
ctrl.usIndex = usIndex;

ctrl.Up = Up;
ctrl.Uf = Uf;

ctrl.Yp = Yp;
ctrl.Yf = Yf;

ctrl.Q = s.Q;
ctrl.R = s.R;

ctrl.Qbar = Qbar;
ctrl.Rbar = Rbar;

ctrl.Ax = Ax;
ctrl.Au = Au;

ctrl.Tx = Tx;
ctrl.Tu = Tu;

ctrl.ExTerm = ExTerm;
ctrl.EuTerm = EuTerm;

ctrl.AtermX = AtermX;
ctrl.AtermU = AtermU;

ctrl.Wy = Wy;
ctrl.WtermX = WtermX;

ctrl.lambdaTerminalX = lambdaTerminalX;
ctrl.lambdaTerminalU = lambdaTerminalU;

ctrl.sigmaY = sigmaY;
ctrl.sigmaEffective = sigmaEffective;

ctrl.lambdaG = s.lambdaG;
ctrl.lambdaSigma = s.lambdaSigma;

ctrl.H = H;

ctrl.Aeq = Aeq;

ctrl.Aineq = Aineq;
ctrl.bineq = bineq;

ctrl.uMax = s.uMax;
ctrl.qMax = s.qMax;

ctrl.Sq = Sq;
ctrl.SqBar = SqBar;

ctrl.opts = optimoptions( ...
    'quadprog', ...
    'Display','off', ...
    'Algorithm','interior-point-convex', ...
    'MaxIterations',1000, ...
    'ConstraintTolerance',1e-7, ...
    'OptimalityTolerance',1e-7);

fprintf('\nDEEPC -- NOISE-AWARE TERMINAL FROBENIUS PENALTY\n');
fprintf('------------------------------------------------\n');
fprintf('Tini                   : %d\n',Tini);
fprintf('N                      : %d\n',N);
fprintf('Terminal window        : %d\n',nTerm);
fprintf('Terminal state weight  : %.3e\n',lambdaTerminalX);
fprintf('Terminal input weight  : %.3e\n',lambdaTerminalU);
fprintf('g variables            : %d\n',ng);
fprintf('Slack variables        : %d\n',nSigma);
fprintf('Steady-input variables : 1\n');
fprintf('Total QP variables     : %d\n',nz);
fprintf('No hard terminal equality used.\n\n');

end