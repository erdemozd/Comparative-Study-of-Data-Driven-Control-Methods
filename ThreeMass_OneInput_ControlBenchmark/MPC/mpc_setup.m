function ctrl = mpc_setup(p)
% MPC_SETUP
%
% Three-mass nominal MPC with internally optimized steady input.
%
% Control objective:
%
%   - q3 tracks r
%   - q1 and q2 positions are NOT tracked during the transient
%   - all velocities are regulated
%
% Known terminal equilibrium:
%
%   x_r = [r; 0; r; 0; r; 0]
%
% Unknown steady input:
%
%   u_s
%
% Decision vector:
%
%   z = [U; u_s]
%
% Cost:
%
%   ||X-Xref||_Q^2
%   +
%   ||U-1*u_s||_R^2
%
% NO analytical u_eq is supplied.

%% 1. Settings

s = benchmark_settings(p);

[A,B,C,D] = ...
    three_mass_nominal_linear(p);

sysd = ...
    c2d( ...
    ss(A,B,C,D), ...
    p.Ts, ...
    'zoh');

Ad = sysd.A;
Bd = sysd.B;

nx = 6;
nu = 1;

N = s.N;

%% Terminal equilibrium window

nTerm = ...
    min(1,N);

%% 2. Lifted prediction

[Sx,Su] = ...
    make_lifted_state_map( ...
    Ad, ...
    Bd, ...
    N);

Qbar = ...
    kron( ...
    eye(N), ...
    s.Q);

Rbar = ...
    kron( ...
    eye(N), ...
    s.R);

%% 3. Decision vector
%
% z = [U; u_s]

nU = N;

nz = ...
    nU + 1;

uIndex = ...
    1:nU;

usIndex = ...
    nU+1;

%% 4. Input deviation map
%
% U - 1_N*u_s

Eu = ...
    ones(N,1);

Au = [ ...
    eye(N), ...
    -Eu];

%% 5. State prediction map
%
% X =
%   Sx*xk + Su*U
%
% Dependence on decision variable:
%
%   [Su  0] * z

Ax = [ ...
    Su, ...
    zeros(nx*N,1)];

%% 6. Hessian

H = ...
    2*( ...
    Ax'*Qbar*Ax ...
    + ...
    Au'*Rbar*Au);

H = ...
    0.5*(H+H');

%% 7. Terminal selectors

Tx = [ ...
    zeros(nx*nTerm,nx*(N-nTerm)), ...
    eye(nx*nTerm)];

Tu = [ ...
    zeros(nTerm,N-nTerm), ...
    eye(nTerm)];

ExTerm = ...
    kron( ...
    ones(nTerm,1), ...
    eye(nx));

EuTerm = ...
    ones(nTerm,1);

%% Terminal input equality
%
% Tu*U = 1*u_s

AeqTerminalU = [ ...
    Tu, ...
    -EuTerm];

%% 8. Physical selectors

Sq = [ ...
    1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];

SqBar = ...
    kron( ...
    eye(N), ...
    Sq);

%% 9. Inequality matrices

% Future input:
%
% |U| <= uMax

Ainput = [ ...
     eye(N), zeros(N,1);
    -eye(N), zeros(N,1)];

binput = ...
    s.uMax*ones(2*N,1);

% Future positions:
%
% |Sq*X| <= qMax
%
% affine state offset handled online.

Aposition = [ ...
     SqBar*Su, zeros(3*N,1);
    -SqBar*Su, zeros(3*N,1)];

% Steady input:
%
% |u_s| <= uMax

AsteadyInput = [ ...
    zeros(1,N),  1;
    zeros(1,N), -1];

bsteadyInput = ...
    s.uMax*ones(2,1);

Aineq = [ ...
    Ainput;
    Aposition;
    AsteadyInput];

%% 10. Store

ctrl.Ad = Ad;
ctrl.Bd = Bd;

ctrl.nx = nx;
ctrl.nu = nu;

ctrl.N = N;
ctrl.Tini = s.Tini;

ctrl.nTerm = nTerm;

ctrl.Sx = Sx;
ctrl.Su = Su;

ctrl.Q = s.Q;
ctrl.R = s.R;

ctrl.Qbar = Qbar;
ctrl.Rbar = Rbar;

ctrl.Ax = Ax;
ctrl.Au = Au;

ctrl.H = H;

ctrl.Tx = Tx;
ctrl.Tu = Tu;

ctrl.ExTerm = ExTerm;
ctrl.EuTerm = EuTerm;

ctrl.AeqTerminalU = AeqTerminalU;

ctrl.Sq = Sq;
ctrl.SqBar = SqBar;

ctrl.Aineq = Aineq;

ctrl.binput = binput;
ctrl.bsteadyInput = bsteadyInput;

ctrl.nz = nz;

ctrl.uIndex = uIndex;
ctrl.usIndex = usIndex;

ctrl.uMax = s.uMax;
ctrl.qMax = s.qMax;

ctrl.opts = ...
    optimoptions( ...
    'quadprog', ...
    'Display','off', ...
    'Algorithm','interior-point-convex', ...
    'MaxIterations',1000, ...
    'ConstraintTolerance',1e-8, ...
    'OptimalityTolerance',1e-8);

fprintf('\n');
fprintf('MPC -- OPTIMIZED STEADY INPUT\n');
fprintf('-----------------------------\n');
fprintf('Prediction horizon     : %d\n',N);
fprintf('Terminal window        : %d\n',nTerm);
fprintf('Decision variables     : %d\n',nz);
fprintf('No analytical u_eq supplied.\n\n');

end