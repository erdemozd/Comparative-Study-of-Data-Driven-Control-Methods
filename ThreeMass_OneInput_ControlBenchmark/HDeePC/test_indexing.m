%% Verify HDeePC horizon alignment

j = 10;   % choose any valid Hankel column

Tini = ctrl.Tini;

% Raw offline data
uData  = deepcData.u;
xData  = deepcData.y;

% Physical ordering:
% [q1; v1; q2; v2; q3; v3]

xKData = xData(1:2,:);
yuData = xData(3:6,:);

%% Corresponding time index

% For Hankel column j:
%
% past samples:
%   j ... j+Tini-1
%
% first future sample:
%   j+Tini

kFuture = j + Tini;

fprintf('\nHDeePC horizon-alignment test\n');
fprintf('-----------------------------\n');
fprintf('Hankel column       : %d\n',j);
fprintf('First future index  : %d\n\n',kFuture);

%% 1. Check first future Yf block

Yf_first = ...
    ctrl.Yf(1:4,j);

yu_expected = ...
    yuData(:,kFuture);

fprintf('First Yf block error:\n');
fprintf('||Yf_1(:,j) - yu(:,kFuture)|| = %.3e\n', ...
    norm(Yf_first-yu_expected));

%% 2. Check first future Uf block

Uf_first = ...
    ctrl.Uf(1,j);

u_expected = ...
    uData(:,kFuture);

fprintf('\nFirst Uf block error:\n');
fprintf('||Uf_1(:,j) - u(:,kFuture)|| = %.3e\n', ...
    norm(Uf_first-u_expected));

%% 3. Check complete first Gfull prediction block

% Select exactly one Hankel trajectory:
g = zeros(ctrl.ng,1);
g(j) = 1;

% Known state at the same first-future time
xK0 = ...
    xKData(:,kFuture);

Xpred = ...
    ctrl.Sfull*xK0 ...
    + ...
    ctrl.Gfull*g;

xFirst = ...
    Xpred(1:6);

xExpected = ...
    xData(:,kFuture);

fprintf('\nComplete first prediction block:\n');
fprintf('||Xpred_1 - x(:,kFuture)|| = %.3e\n', ...
    norm(xFirst-xExpected));

fprintf('\nPredicted first block:\n');
disp(xFirst);

fprintf('Expected raw-data block:\n');
disp(xExpected);