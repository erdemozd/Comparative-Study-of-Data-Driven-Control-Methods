function ctrl = hdeepc_setup(p,deepcData)
% HDEEPC_SETUP
%
% Three-mass QP HDeePC.
%
% Physical state ordering used by the benchmark:
%
%   x = [q1; v1; q2; v2; q3; v3]
%
% HDeePC partition:
%
%   x_kappa = [q1; v1]
%   x_u     = [q2; v2; q3; v3]
%
% Unknown/output subsystem:
%
%   y_u = x_u = [q2; v2; q3; v3]
%
% Known/output subsystem:
%
%   y_kappa = x_kappa = [q1; v1]
%
% Hence:
%
%   C_u     = I4
%   C_f     = 0
%   C_c     = 0
%   C_kappa = I2
%
% and therefore:
%
%   A_y = A_c
%   C_y = 0
%
% Only u_s is optimized as the unknown steady quantity.
%
% Decision:
%
%   z = [g; sigma; u_s]

%% 1. Settings

s = benchmark_settings(p);

Tini = ...
    deepcData.Tini;

N = ...
    deepcData.N;

L = ...
    Tini + N;

nTerm = ...
    min(1,N);

%% 2. Behavioral data

uData = ...
    deepcData.u;

% Physical data ordering:
%
%   [q1; v1; q2; v2; q3; v3]
%
% HDeePC unknown output:
%
%   y_u = [q2; v2; q3; v3]

yuData = ...
    deepcData.y(3:6,:);

Hu = ...
    block_hankel( ...
    uData, ...
    L);

Hy = ...
    block_hankel( ...
    yuData, ...
    L);

Up = ...
    Hu(1:Tini,:);

Uf = ...
    Hu(Tini+1:end,:);

Yp = ...
    Hy(1:4*Tini,:);

Yf = ...
    Hy(4*Tini+1:end,:);

ng = ...
    size(Hu,2);

%% 3. Full nominal discrete-time model
%
% IMPORTANT:
%
% Do NOT discretize the known mass-1 subsystem separately here.
%
% First form the complete continuous nominal model, discretize the
% six-state system jointly, then extract the HDeePC blocks.
%
% Original physical ordering:
%
%   x = [x_kappa; x_u]
%
%     = [q1; v1; q2; v2; q3; v3]
%
% Paper ordering:
%
%   x_bar = [x_u; x_kappa]
%
% The blocks required in the paper are therefore obtained directly
% from the full discrete model.

AcFull = [ ...
    0, ...
    1, ...
    0, ...
    0, ...
    0, ...
    0;
    ...
    -(p.k1+p.k12)/p.m1, ...
    -(p.c1+p.c12)/p.m1, ...
    p.k12/p.m1, ...
    p.c12/p.m1, ...
    0, ...
    0;
    ...
    0, ...
    0, ...
    0, ...
    1, ...
    0, ...
    0;
    ...
    p.k12/p.m2, ...
    p.c12/p.m2, ...
    -(p.k12+p.k23)/p.m2, ...
    -(p.c12+p.c23)/p.m2, ...
    p.k23/p.m2, ...
    p.c23/p.m2;
    ...
    0, ...
    0, ...
    0, ...
    0, ...
    0, ...
    1;
    ...
    0, ...
    0, ...
    p.k23/p.m3, ...
    p.c23/p.m3, ...
    -p.k23/p.m3, ...
    -p.c23/p.m3];

BcFull = [ ...
    0;
    1/p.m1;
    0;
    0;
    0;
    0];

sysFull = ...
    c2d( ...
    ss( ...
    AcFull, ...
    BcFull, ...
    eye(6), ...
    zeros(6,1)), ...
    p.Ts, ...
    'zoh');

Ad = ...
    sysFull.A;

Bd = ...
    sysFull.B;

%% 3a. HDeePC state partition
%
% Original benchmark ordering:
%
%   x = [x_kappa; x_u]
%
% Therefore:
%
%   A_d =
%
%   [ A_kappa   A_c
%     A_f       A_u ]
%
% whereas the paper writes:
%
%   [ A_u       A_f
%     A_c       A_kappa ]
%
% We can extract the paper blocks directly without explicitly
% permuting the full matrix.

Ak = ...
    Ad(1:2,1:2);

Ac = ...
    Ad(1:2,3:6);

Af = ...
    Ad(3:6,1:2);

Au = ...
    Ad(3:6,3:6);

Bk = ...
    Bd(1:2);

Bu = ...
    Bd(3:6);

%% 3b. Output partition
%
% y_u = x_u
% y_kappa = x_kappa

Cu = ...
    eye(4);

Cf = ...
    zeros(4,2);

Cc = ...
    zeros(2,4);

Ck = ...
    eye(2);

Du = ...
    zeros(4,1);

Dk = ...
    zeros(2,1);

%% 3c. Paper transformation matrices
%
% Paper conditions:
%
%   A_c = A_y C_u
%   A_y C_f = 0
%   A_y D_u = 0
%
% Since C_u = I4:
%
%   A_y = A_c
%
% Likewise:
%
%   C_c = C_y C_u
%
% Since C_c = 0 and C_u = I4:
%
%   C_y = 0

Ay = ...
    Ac;

Cy = ...
    zeros(2,4);

%% Optional consistency checks

tol = 1e-10;

if norm(Ac - Ay*Cu,'fro') > tol
    error('HDeePC consistency failure: Ac ~= Ay*Cu.');
end

if norm(Ay*Cf,'fro') > tol
    error('HDeePC consistency failure: Ay*Cf ~= 0.');
end

if norm(Ay*Du,'fro') > tol
    error('HDeePC consistency failure: Ay*Du ~= 0.');
end

if norm(Cc - Cy*Cu,'fro') > tol
    error('HDeePC consistency failure: Cc ~= Cy*Cu.');
end

if norm(Cy*Cf,'fro') > tol
    error('HDeePC consistency failure: Cy*Cf ~= 0.');
end

if norm(Cy*Du,'fro') > tol
    error('HDeePC consistency failure: Cy*Du ~= 0.');
end

%% 4. Full-state hybrid prediction
%
% Physical prediction ordering retained for the benchmark:
%
%   x =
%   [q1; v1; q2; v2; q3; v3]
%
% Therefore:
%
%   X =
%   Sfull*xK0 + Gfull*g
%
% with
%
%   x_kappa(k+1)
%       = Ak*x_kappa(k)
%       + Ay*y_u(k)
%       + Bk*u(k)
%
% and
%
%   y_u(k) = Yf_block*g

Gfull = ...
    zeros(6*N,ng);

Sfull = ...
    zeros(6*N,2);

Kmap = ...
    zeros(2,ng);

Smap = ...
    eye(2);

for i = 1:N

    rowsFull = ...
        (i-1)*6 ...
        + ...
        (1:6);

    rowsYu = ...
        (i-1)*4 ...
        + ...
        (1:4);

    % Known part:
    %
    % x_kappa = [q1; v1]

    Gfull(rowsFull(1:2),:) = ...
        Kmap;

    Sfull(rowsFull(1:2),:) = ...
        Smap;

    % Unknown/data-driven part:
    %
    % y_u = [q2; v2; q3; v3]

    Gfull(rowsFull(3:6),:) = ...
        Yf(rowsYu,:);

    % Future input at this prediction step

    ui = ...
        Uf(i,:);

    % Paper-compatible known subsystem recursion:
    %
    % x_kappa(k+1)
    % =
    % Ak*x_kappa(k)
    % + Ay*y_u(k)
    % + Bk*u(k)

    Kmap = ...
        Ak*Kmap ...
        + ...
        Ay*Yf(rowsYu,:) ...
        + ...
        Bk*ui;

    Smap = ...
        Ak*Smap;

end

%% 5. Decision vector

nSigma = ...
    4*Tini;

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

%% 6. Cost

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

Ax = [ ...
    Gfull, ...
    zeros(6*N,nSigma), ...
    zeros(6*N,1)];

AuCost = [ ...
    Uf, ...
    zeros(N,nSigma), ...
    -Eu];

Hreg = ...
    zeros(nz,nz);

Hreg(gIndex,gIndex) = ...
    2*s.lambdaG*eye(ng);

Hreg(sigmaIndex,sigmaIndex) = ...
    2*s.lambdaSigma*eye(nSigma);

H = ...
    2*( ...
    Ax'*Qbar*Ax ...
    + ...
    AuCost'*Rbar*AuCost) ...
    + ...
    Hreg;

H = ...
    0.5*(H+H');

%% 7. Past constraints

AeqPastInput = [ ...
    Up, ...
    zeros(Tini,nSigma+1)];

AeqPastOutput = [ ...
    Yp, ...
    -eye(nSigma), ...
    zeros(nSigma,1)];

%% 8. Terminal selectors

Tx = [ ...
    zeros(6*nTerm,6*(N-nTerm)), ...
    eye(6*nTerm)];

Tu = [ ...
    zeros(nTerm,N-nTerm), ...
    eye(nTerm)];

ExTerm = ...
    kron( ...
    ones(nTerm,1), ...
    eye(6));

EuTerm = ...
    ones(nTerm,1);

%% Terminal full physical state

AeqTerminalX = [ ...
    Tx*Gfull, ...
    zeros(6*nTerm,nSigma), ...
    zeros(6*nTerm,1)];

%% Terminal input = u_s

AeqTerminalU = [ ...
    Tu*Uf, ...
    zeros(nTerm,nSigma), ...
    -EuTerm];

Aeq = [ ...
    AeqPastInput;
    AeqPastOutput;
    AeqTerminalX;
    AeqTerminalU];

%% 9. Physical constraints

Sq = [ ...
    1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];

SqBar = ...
    kron( ...
    eye(N), ...
    Sq);

Ainput = [ ...
     Uf, zeros(N,nSigma+1);
    -Uf, zeros(N,nSigma+1)];

ApositionG = ...
    SqBar*Gfull;

Aposition = [ ...
     ApositionG, zeros(3*N,nSigma+1);
    -ApositionG, zeros(3*N,nSigma+1)];

AsteadyInput = [ ...
    zeros(1,ng+nSigma),  1;
    zeros(1,ng+nSigma), -1];

Aineq = [ ...
    Ainput;
    Aposition;
    AsteadyInput];

ctrl.binput = ...
    s.uMax*ones(2*N,1);

ctrl.bsteadyInput = ...
    s.uMax*ones(2,1);

%% 10. Store

ctrl.Tini = Tini;
ctrl.N = N;
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

ctrl.Gfull = Gfull;
ctrl.Sfull = Sfull;

ctrl.Ax = Ax;
ctrl.Au = AuCost;

ctrl.Qbar = Qbar;
ctrl.Rbar = Rbar;

ctrl.H = H;

ctrl.Aeq = Aeq;

ctrl.Aineq = Aineq;

ctrl.Tx = Tx;
ctrl.ExTerm = ExTerm;

ctrl.Sq = Sq;
ctrl.SqBar = SqBar;

ctrl.uMax = s.uMax;
ctrl.qMax = s.qMax;

%% Store full nominal model

ctrl.Ad = Ad;
ctrl.Bd = Bd;

%% Store HDeePC state partition

ctrl.Ak = Ak;
ctrl.Ac = Ac;
ctrl.Af = Af;
ctrl.AuModel = Au;

ctrl.Bk = Bk;
ctrl.Bu = Bu;

%% Store output partition

ctrl.Cu = Cu;
ctrl.Cf = Cf;
ctrl.Cc = Cc;
ctrl.Ck = Ck;

ctrl.Du = Du;
ctrl.Dk = Dk;

%% Store transformed paper matrices

ctrl.Ay = Ay;
ctrl.Cy = Cy;

ctrl.opts = ...
    optimoptions( ...
    'quadprog', ...
    'Display','off', ...
    'Algorithm','interior-point-convex', ...
    'MaxIterations',1000, ...
    'ConstraintTolerance',1e-7, ...
    'OptimalityTolerance',1e-7);

fprintf('\n');
fprintf('HDeePC-QP -- PAPER-COMPATIBLE PARTITION\n');
fprintf('---------------------------------------\n');
fprintf('Terminal window    : %d\n',nTerm);
fprintf('Total QP variables : %d\n',nz);
fprintf('Known state        : x_kappa = [q1; v1]\n');
fprintf('Unknown output     : y_u     = [q2; v2; q3; v3]\n');
fprintf('Using Ay = Ac from full discrete model.\n');
fprintf('Using Cy = 0 because y_kappa = x_kappa.\n');
fprintf('No analytical u_eq supplied.\n\n');

fprintf('Ak =\n');
disp(Ak);

fprintf('Ac = Ay =\n');
disp(Ac);

fprintf('Bk =\n');
disp(Bk);

fprintf('Cy =\n');
disp(Cy);

end