function ctrl = deepc_setup(p,deepcData)
% DEEPC_SETUP
%
% Standard full-state DeePC with internally optimized steady input.
%
% Running objective:
%
%   q3 position tracking only
%   velocity regulation
%
% q1 and q2 position weights remain zero.
%
% Known terminal equilibrium geometry:
%
%   [q1 v1 q2 v2 q3 v3]
%
%   =
%
%   [r 0 r 0 r 0]
%
% Unknown:
%
%   u_s
%
% Decision:
%
%   z = [g; sigma; u_s]

%% 1. Settings

s = benchmark_settings(p);

nu = 1;
ny = 6;

Tini = ...
    deepcData.Tini;

N = ...
    deepcData.N;

L = ...
    Tini+N;

nTerm = ...
    min(1,N);

%% 2. Hankel matrices

Hu = ...
    block_hankel( ...
    deepcData.u, ...
    L);

Hy = ...
    block_hankel( ...
    deepcData.y, ...
    L);

Up = ...
    Hu(1:nu*Tini,:);

Uf = ...
    Hu(nu*Tini+1:end,:);

Yp = ...
    Hy(1:ny*Tini,:);

Yf = ...
    Hy(ny*Tini+1:end,:);

ng = ...
    size(Hu,2);

%% 3. Decision vector

nSigma = ...
    ny*Tini;

nz = ...
    ng ...
    + ...
    nSigma ...
    + ...
    1;

gIndex = ...
    1:ng;

sigmaIndex = ...
    ng+(1:nSigma);

usIndex = ...
    ng+nSigma+1;

%% 4. Cost

Qbar = ...
    kron( ...
    eye(N), ...
    s.Q);

Rbar = ...
    kron( ...
    eye(N), ...
    s.R);

Eu = ...
    ones(N,1);

%% State trajectory map

Ax = [ ...
    Yf, ...
    zeros(ny*N,nSigma), ...
    zeros(ny*N,1)];

%% Input deviation:
%
% Uf*g - 1*u_s

Au = [ ...
    Uf, ...
    zeros(N,nSigma), ...
    -Eu];

%% Regularization

Hreg = ...
    zeros(nz,nz);

Hreg(gIndex,gIndex) = ...
    2*s.lambdaG*eye(ng);

Hreg(sigmaIndex,sigmaIndex) = ...
    2*s.lambdaSigma*eye(nSigma);

%% Hessian

H = ...
    2*( ...
    Ax'*Qbar*Ax ...
    + ...
    Au'*Rbar*Au) ...
    + ...
    Hreg;

H = ...
    0.5*(H+H');

%% 5. Past equality constraints

AeqInput = [ ...
    Up, ...
    zeros(nu*Tini,nSigma), ...
    zeros(nu*Tini,1)];

AeqOutput = [ ...
    Yp, ...
    -eye(nSigma), ...
    zeros(nSigma,1)];

%% 6. Terminal selectors

Tx = [ ...
    zeros(ny*nTerm,ny*(N-nTerm)), ...
    eye(ny*nTerm)];

Tu = [ ...
    zeros(nTerm,N-nTerm), ...
    eye(nTerm)];

ExTerm = ...
    kron( ...
    ones(nTerm,1), ...
    eye(ny));

EuTerm = ...
    ones(nTerm,1);

%% Terminal state:
%
% Tx*Yf*g = repeated xTarget

AeqTerminalX = [ ...
    Tx*Yf, ...
    zeros(ny*nTerm,nSigma), ...
    zeros(ny*nTerm,1)];

%% Terminal input:
%
% Tu*Uf*g = 1*u_s

AeqTerminalU = [ ...
    Tu*Uf, ...
    zeros(nTerm,nSigma), ...
    -EuTerm];

Aeq = [ ...
    AeqInput;
    AeqOutput;
    AeqTerminalX;
    AeqTerminalU];

%% 7. Physical constraints

Sq = [ ...
    1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];

SqBar = ...
    kron( ...
    eye(N), ...
    Sq);

Yposition = ...
    SqBar*Yf;

%% Future input

Ainput = [ ...
     Uf, zeros(N,nSigma+1);
    -Uf, zeros(N,nSigma+1)];

binput = ...
    s.uMax*ones(2*N,1);

%% Future positions

Aposition = [ ...
     Yposition, zeros(3*N,nSigma+1);
    -Yposition, zeros(3*N,nSigma+1)];

bposition = ...
    s.qMax*ones(6*N,1);

%% Steady input

AsteadyInput = [ ...
    zeros(1,ng+nSigma),  1;
    zeros(1,ng+nSigma), -1];

bsteadyInput = ...
    s.uMax*ones(2,1);

Aineq = [ ...
    Ainput;
    Aposition;
    AsteadyInput];

bineq = [ ...
    binput;
    bposition;
    bsteadyInput];

%% 8. Store

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

ctrl.lambdaG = s.lambdaG;
ctrl.lambdaSigma = s.lambdaSigma;

ctrl.H = H;

ctrl.Aeq = Aeq;

ctrl.Aineq = Aineq;
ctrl.bineq = bineq;

ctrl.Tx = Tx;
ctrl.Tu = Tu;

ctrl.ExTerm = ExTerm;
ctrl.EuTerm = EuTerm;

ctrl.uMax = s.uMax;
ctrl.qMax = s.qMax;

ctrl.Sq = Sq;
ctrl.SqBar = SqBar;

ctrl.opts = ...
    optimoptions( ...
    'quadprog', ...
    'Display','off', ...
    'Algorithm','interior-point-convex', ...
    'MaxIterations',1000, ...
    'ConstraintTolerance',1e-7, ...
    'OptimalityTolerance',1e-7);

fprintf('\n');
fprintf('DEEPC -- OPTIMIZED STEADY INPUT\n');
fprintf('-------------------------------\n');

fprintf('Tini                   : %d\n',Tini);
fprintf('N                      : %d\n',N);
fprintf('Terminal window        : %d\n',nTerm);
fprintf('g variables            : %d\n',ng);
fprintf('Slack variables        : %d\n',nSigma);
fprintf('Steady-input variables : 1\n');
fprintf('Total QP variables     : %d\n',nz);
fprintf('No analytical u_eq supplied.\n\n');

end