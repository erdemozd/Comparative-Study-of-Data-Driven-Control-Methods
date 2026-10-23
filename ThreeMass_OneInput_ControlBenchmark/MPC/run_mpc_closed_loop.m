clear; clc; close all;

% Select 'clean' or 'noisy'. Noisy mode uses the DeePC training-data
% per-channel sensor standard deviations (without a state estimator).
measurementMode = 'noisy';
measurementNoiseSeed = 42;

p = three_mass_params();
s = benchmark_settings(p);
ctrl = mpc_setup(p);

projectRoot = fileparts(fileparts(mfilename('fullpath')));
if isempty(projectRoot) || ~exist(fullfile(projectRoot,'results'),'dir')
    projectRoot = pwd;
end
resultsFolder = fullfile(projectRoot,'results');
if ~exist(resultsFolder,'dir'), mkdir(resultsFolder); end

t = (0:p.Ts:s.Tend)';
Ns = numel(t);
r = build_reference(t);

switch lower(measurementMode)
    case 'clean'
        sigmaY = zeros(6,1);
        measurementNoise = zeros(6,Ns);
    case 'noisy'
        dataFile = fullfile(projectRoot,'data','deepc_training_data.mat');
        if ~exist(dataFile,'file')
            error('Run DeePC/generate_deepc_data.m first, or select clean MPC.');
        end
        D = load(dataFile,'deepcData');
        if ~isfield(D,'deepcData') || ~isfield(D.deepcData,'sigmaY')
            error('deepcData.sigmaY is required. Regenerate the DeePC training data.');
        end
        sigmaY = D.deepcData.sigmaY(:);
        validateattributes(sigmaY,{'numeric'}, ...
            {'real','finite','nonnegative','numel',6},mfilename,'sigmaY');
        noiseStream = RandStream('mt19937ar','Seed',measurementNoiseSeed);
        measurementNoise = sigmaY .* randn(noiseStream,6,Ns);
    otherwise
        error('Unknown measurementMode: %s. Use clean or noisy.',measurementMode);
end

x = zeros(6,Ns);
yMeasured = zeros(6,Ns);
u = zeros(Ns,1);
solveTime = nan(Ns-1,1);
Jlog = nan(Ns-1,1);
uSteady = nan(Ns-1,1);
fail = false(Ns-1,1);
predNext = nan(6,Ns-1);
oneStepErr = nan(6,Ns-1);

for k = 1:Ns-1
    rPreview = repmat(r(k),ctrl.N,1);
    rDotPreview = zeros(ctrl.N,1);

    yMeasured(:,k) = x(:,k) + measurementNoise(:,k);
    [uc,info] = mpc_control(yMeasured(:,k),rPreview,rDotPreview,p,ctrl);

    if info.success
        uk = uc;
        Jlog(k) = info.J;
        if isfield(info,'uSteady'), uSteady(k) = info.uSteady; end
        if isfield(info,'xPred') && ~isempty(info.xPred) && size(info.xPred,2) >= 2
            predNext(:,k) = info.xPred(:,2);
        end
    else
        fail(k) = true;
        uk = 0;
        if k > 1, uk = u(k-1); end
    end

    uk = min(max(uk,-ctrl.uMax),ctrl.uMax);
    u(k) = uk;
    solveTime(k) = info.solveTime;

    x(:,k+1) = rk4_step(x(:,k),uk,p.Ts,p);
    if all(isfinite(predNext(:,k)))
        oneStepErr(:,k) = x(:,k+1)-predNext(:,k);
    end
end
u(end) = u(end-1);
yMeasured(:,end) = x(:,end) + measurementNoise(:,end);

q3 = x(5,:)';
rmse = sqrt(mean((q3-r).^2));

dq23 = x(5,:)'-x(3,:)';
Flinear23 = p.k23*dq23;
Fcubic23 = p.k3*dq23.^3;
nonlinearRatio = 100*abs(Fcubic23)./(abs(Flinear23)+1e-12);

fprintf('\nMPC DIAGNOSTIC SUMMARY\n');
fprintf('Measurement mode         : %s\n',measurementMode);
fprintf('q3 RMSE                  : %.6f m\n',rmse);
fprintf('QP failures              : %d\n',sum(fail));
fprintf('Mean solve time          : %.3f ms\n',1e3*mean(solveTime,'omitnan'));
fprintf('Maximum solve time       : %.3f ms\n',1e3*max(solveTime,[],'omitnan'));
fprintf('Deadline misses (> Ts)   : %d / %d\n',sum(solveTime>p.Ts),Ns-1);
fprintf('Max nonlinear ratio      : %.2f %%\n\n',max(nonlinearRatio));

% Figure 1: closed-loop signals
figure('Color','w','Name','MPC - Closed-Loop Diagnostics');
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(t,r,'k--','LineWidth',1.4); hold on; plot(t,q3,'LineWidth',1.5); grid on; ylabel('q_3 [m]'); title(sprintf('Tracking, RMSE = %.4f m',rmse)); legend('r','q_3','Location','best');
nexttile; plot(t,x(1,:)','LineWidth',1.1); hold on; plot(t,x(3,:)','LineWidth',1.1); plot(t,x(5,:)','LineWidth',1.3); grid on; ylabel('Position [m]'); title('All positions'); legend('q_1','q_2','q_3','Location','best');
nexttile; plot(t,x(2,:)','LineWidth',1.1); hold on; plot(t,x(4,:)','LineWidth',1.1); plot(t,x(6,:)','LineWidth',1.1); grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]'); title('Velocities'); legend('v_1','v_2','v_3','Location','best');
nexttile; plot(t,u,'LineWidth',1.3); hold on; plot(t(1:end-1),uSteady,'LineWidth',1.1); plot(t,p.k1*r,'k--','LineWidth',1.0); yline(ctrl.uMax,'--'); yline(-ctrl.uMax,'--'); grid on; xlabel('Time [s]'); ylabel('Force [N]'); title('Applied and optimized steady input'); legend('u','u_s','k_1r diagnostic','Location','best');
title(tl,sprintf('Nominal MPC (%s measurements)',measurementMode));

% Figure 2: prediction + computation
figure('Color','w','Name','MPC - Prediction and Computation');
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(t(2:end),x(5,2:end)','LineWidth',1.3); hold on; plot(t(2:end),predNext(5,:)','--','LineWidth',1.2); grid on; ylabel('q_3 [m]'); title('One-step prediction'); legend('Actual','Predicted','Location','best');
nexttile; semilogy(t(1:end-1),max(vecnorm(oneStepErr,2,1)',1e-12),'LineWidth',1.2); grid on; ylabel('||e_{1-step}||_2'); title('One-step state prediction error');
nexttile; plot(t(1:end-1),1e3*solveTime,'LineWidth',1.2); hold on; yline(1e3*p.Ts,'k--'); grid on; xlabel('Time [s]'); ylabel('Solve time [ms]'); title('QP computation load');
nexttile; plot(t(1:end-1),Jlog,'LineWidth',1.2); grid on; xlabel('Time [s]'); ylabel('J^*'); title('Optimal objective value');
title(tl,'MPC Prediction and Optimization');

% Figure 3: nonlinearity exposure
figure('Color','w','Name','MPC - Plant Nonlinearity');
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile; plot(t,dq23,'LineWidth',1.3); grid on; ylabel('q_3-q_2 [m]'); title('Nonlinear spring deformation');
nexttile; plot(t,Flinear23,'LineWidth',1.1); hold on; plot(t,Fcubic23,'LineWidth',1.1); plot(t,nonlinearRatio,'--','LineWidth',1.1); grid on; xlabel('Time [s]'); title('Spring-force components / nonlinear exposure'); legend('Linear force [N]','Cubic force [N]','Cubic/linear [%]','Location','best');
title(tl,'True Plant Nonlinearity');

result = struct();
result.t=t; result.x=x; result.u=u; result.r=r; result.rmse=rmse;
result.measurementMode=measurementMode;
result.measurementNoiseSeed=measurementNoiseSeed;
result.sigmaY=sigmaY; result.measurementNoise=measurementNoise;
result.yMeasured=yMeasured;
result.solveTime=solveTime; result.J=Jlog; result.uSteady=uSteady; result.fail=fail;
result.predNext=predNext; result.oneStepErr=oneStepErr;
result.dq23=dq23; result.Flinear23=Flinear23; result.Fcubic23=Fcubic23; result.nonlinearRatio=nonlinearRatio;
result.ctrl=ctrl; result.p=p;
save(fullfile(resultsFolder,'mpc_three_mass_diagnostics.mat'),'result');
