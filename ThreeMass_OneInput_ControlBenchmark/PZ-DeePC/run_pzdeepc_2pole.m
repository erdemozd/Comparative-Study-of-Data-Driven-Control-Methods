clear;
clc;
close all;

% Output sensor noise: offline predictor data and online feedback are independent.
% Applied training inputs are retained in both offline modes.
offlineDataMode = 'noisy';  % 'clean' or 'noisy'
measurementMode = 'noisy';  % 'clean' or 'noisy'
measurementNoiseSeed = 42;

p = three_mass_params();
s = benchmark_settings(p);

projectRoot = fileparts(fileparts(mfilename('fullpath')));
dataFile = fullfile(projectRoot,'data','deepc_training_data.mat');

if ~exist(dataFile,'file')
    error('Run DeePC/generate_deepc_data.m first.');
end

load(dataFile,'deepcData');
switch lower(offlineDataMode)
    case 'clean'
        if ~isfield(deepcData,'yClean')
            error('Regenerate DeePC training data to obtain yClean.');
        end
        deepcData.y = deepcData.yClean;
    case 'noisy'
        % Use the recorded noisy offline outputs.
    otherwise
        error('offlineDataMode must be clean or noisy.');
end


ctrl = pzdeepc_setup_2pole(p,deepcData);

fprintf('\n2-Pole PZ-DeePC known poles:\n');
disp(ctrl.prior.knownPoles);

t = (0:p.Ts:s.Tend)';
Ns = numel(t);
r = build_reference(t);

x = zeros(6,Ns);
u = zeros(Ns,1);

uBuffer = zeros(1,ctrl.Tini);
yBuffer = zeros(6,ctrl.Tini);

solveTime = nan(Ns-1,1);
sigmaNorm = nan(Ns-1,1);
gNorm = nan(Ns-1,1);
fail = false(Ns-1,1);

% Fixed sensor scale shared with MPC; offline mode does not change this scale.
sigmaY = deepcData.sigmaY(:);
validateattributes(sigmaY,{'numeric'}, ...
    {'real','finite','nonnegative','numel',6},mfilename,'sigmaY');
switch lower(measurementMode)
    case 'clean'
        measurementNoise = zeros(6,Ns);
    case 'noisy'
        noiseStream = RandStream('mt19937ar','Seed',measurementNoiseSeed);
        measurementNoise = sigmaY .* randn(noiseStream,6,Ns);
    otherwise
        error('measurementMode must be clean or noisy.');
end
yMeasured = zeros(6,Ns);
fprintf('Offline data: %s | Online measurements: %s\n',offlineDataMode,measurementMode);

for k = 1:Ns-1
    yMeasured(:,k) = x(:,k) + measurementNoise(:,k);

    rPreview = repmat(r(k),ctrl.N,1);
    rDotPreview = zeros(ctrl.N,1);

    [uc,info] = pzdeepc_control( ...
        uBuffer(:), ...
        yBuffer(:), ...
        rPreview, ...
        rDotPreview, ...
        p, ...
        ctrl);

    if info.success
        uk = uc;
        sigmaNorm(k) = info.sigmaNorm;
        gNorm(k) = norm(info.g,2);
    else
        fail(k) = true;
        if k > 1
            uk = u(k-1);
        else
            uk = 0;
        end
    end

    uk = min(max(uk,-ctrl.uMax),ctrl.uMax);

    u(k) = uk;
    solveTime(k) = info.solveTime;

    uBuffer = [uBuffer(:,2:end),uk];
    yBuffer = [yBuffer(:,2:end),yMeasured(:,k)];

    x(:,k+1) = rk4_step(x(:,k),uk,p.Ts,p);
end

u(end) = u(end-1);
yMeasured(:,end) = x(:,end) + measurementNoise(:,end);

q3 = x(5,:)';
rmse = sqrt(mean((q3-r).^2));

fprintf('\n2-Pole PZ-DeePC q3 RMSE: %.6f m\n',rmse);
fprintf('Failures: %d\n',sum(fail));
fprintf('Mean solve time: %.6f s\n\n',mean(solveTime,'omitnan'));

figure('Color','w','Name','2-Pole PZ-DeePC');
plot(t,r,'k--','LineWidth',1.5);
hold on;
plot(t,q3,'LineWidth',1.5);
grid on;
xlabel('Time [s]');
ylabel('q3 [m]');
legend('r','q3');
title(sprintf('2-Pole PZ-DeePC -- q3 RMSE %.4f m',rmse));

if ~exist(fullfile(projectRoot,'results'),'dir'), mkdir(fullfile(projectRoot,'results')); end
save(fullfile(projectRoot,'results','pzdeepc_2pole_three_mass.mat'), ...
    't','x','u','r','rmse','solveTime','sigmaNorm','gNorm','fail','ctrl','p','offlineDataMode','measurementMode', ...
    'measurementNoiseSeed','sigmaY','measurementNoise','yMeasured');
