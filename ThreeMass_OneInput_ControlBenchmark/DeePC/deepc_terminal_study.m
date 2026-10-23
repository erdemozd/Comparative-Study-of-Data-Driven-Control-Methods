clear; clc; close all;

p = three_mass_params();
s = benchmark_settings(p);

projectRoot = fileparts(fileparts(mfilename('fullpath')));

if isempty(projectRoot)
    projectRoot = pwd;
end

dataFile = fullfile(projectRoot,'data','deepc_training_data.mat');

if ~exist(dataFile,'file')
    error('Run DeePC/generate_deepc_data.m first.');
end

load(dataFile,'deepcData');

ctrlHard = deepc_setup(p,deepcData);
ctrlNoTerminal = deepc_setup_no_terminal(p,deepcData);
ctrlFrobenius = deepc_setup_terminal_frobenius(p,deepcData);

controllers = {
    ctrlHard
    ctrlNoTerminal
    ctrlFrobenius};

controlFunctions = {
    @deepc_control
    @deepc_control_no_terminal
    @deepc_control_terminal_frobenius};

names = {
    'Hard terminal'
    'No terminal'
    'Noise-aware Frobenius'};

t = (0:p.Ts:s.Tend)';
Ns = numel(t);
r = build_reference(t);

nControllers = numel(controllers);

xAll = cell(nControllers,1);
uAll = cell(nControllers,1);
uSteadyAll = cell(nControllers,1);
gNormAll = cell(nControllers,1);
sigmaNormAll = cell(nControllers,1);
solveTimeAll = cell(nControllers,1);
oneStepErrNormAll = cell(nControllers,1);
terminalStateNormAll = cell(nControllers,1);
terminalInputNormAll = cell(nControllers,1);

rmseAll = nan(nControllers,1);
maxNonlinearAll = nan(nControllers,1);

for c = 1:nControllers

    ctrl = controllers{c};
    controlFcn = controlFunctions{c};

    x = zeros(6,Ns);
    u = zeros(Ns,1);

    uBuffer = zeros(1,ctrl.Tini);
    yBuffer = zeros(6,ctrl.Tini);

    solveTime = nan(Ns-1,1);
    sigmaNorm = nan(Ns-1,1);
    gNorm = nan(Ns-1,1);
    uSteady = nan(Ns-1,1);

    predNext = nan(6,Ns-1);
    oneStepErr = nan(6,Ns-1);

    terminalStateNorm = nan(Ns-1,1);
    terminalInputNorm = nan(Ns-1,1);

    for k = 1:Ns-1

        rPreview = repmat(r(k),ctrl.N,1);
        rDotPreview = zeros(ctrl.N,1);

        [uc,info] = controlFcn(uBuffer(:),yBuffer(:),rPreview,rDotPreview,p,ctrl);

        if info.success
            uk = uc;

            sigmaNorm(k) = info.sigmaNorm;

            if isfield(info,'g') && ~isempty(info.g)
                gNorm(k) = norm(info.g,2);
            end

            if isfield(info,'uSteady')
                uSteady(k) = info.uSteady;
            end

            if isfield(info,'xPred') && ~isempty(info.xPred) && size(info.xPred,2)>=2
                predNext(:,k) = info.xPred(:,2);
            end

            if isfield(info,'terminalStateNorm')
                terminalStateNorm(k) = info.terminalStateNorm;
            end

            if isfield(info,'terminalInputNorm')
                terminalInputNorm(k) = info.terminalInputNorm;
            end

        else
            uk = 0;

            if k>1
                uk = u(k-1);
            end
        end

        uk = min(max(uk,-ctrl.uMax),ctrl.uMax);

        u(k) = uk;
        solveTime(k) = info.solveTime;

        uBuffer = [uBuffer(:,2:end),uk];
        yBuffer = [yBuffer(:,2:end),x(:,k)];

        x(:,k+1) = rk4_step(x(:,k),uk,p.Ts,p);

        if all(isfinite(predNext(:,k)))
            oneStepErr(:,k) = x(:,k+1)-predNext(:,k);
        end
    end

    u(end) = u(end-1);

    q3 = x(5,:)';
    rmse = sqrt(mean((q3-r).^2));

    dq23 = x(5,:)'-x(3,:)';
    Flinear23 = p.k23*dq23;
    Fcubic23 = p.k3*dq23.^3;
    nonlinearRatio = 100*abs(Fcubic23)./(abs(Flinear23)+1e-12);

    oneStepErrNorm = vecnorm(oneStepErr,2,1)';

    xAll{c} = x;
    uAll{c} = u;
    uSteadyAll{c} = uSteady;
    gNormAll{c} = gNorm;
    sigmaNormAll{c} = sigmaNorm;
    solveTimeAll{c} = solveTime;
    oneStepErrNormAll{c} = oneStepErrNorm;
    terminalStateNormAll{c} = terminalStateNorm;
    terminalInputNormAll{c} = terminalInputNorm;

    rmseAll(c) = rmse;
    maxNonlinearAll(c) = max(nonlinearRatio);
end

fprintf('\nDEEPC TERMINAL STUDY\n');
fprintf('================================================================================\n');
fprintf('%-25s %10s %12s %12s %12s\n', ...
    'Controller','RMSE','Mean |g|','Mean slack','Solve [ms]');
fprintf('--------------------------------------------------------------------------------\n');

for c = 1:nControllers
    fprintf('%-25s %10.4f %12.3e %12.3e %12.1f\n', ...
        names{c}, ...
        rmseAll(c), ...
        mean(gNormAll{c},'omitnan'), ...
        mean(sigmaNormAll{c},'omitnan'), ...
        1e3*mean(solveTimeAll{c},'omitnan'));
end
%%
fprintf('================================================================================\n');

figure('Color','w','Name','DeePC Terminal Study - Tracking','Position',[100 80 1400 850]);

tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

ax1 = nexttile; hold on; grid on; box on;
ax2 = nexttile; hold on; grid on; box on;
ax3 = nexttile; hold on; grid on; box on;
ax4 = nexttile; hold on; grid on; box on;

plot(ax1,t,r,'k--','LineWidth',1.6,'DisplayName','Reference');

for c = 1:nControllers
    plot(ax1,t,xAll{c}(5,:)','LineWidth',1.3,'DisplayName',names{c});
    plot(ax2,t,uAll{c},'LineWidth',1.2,'DisplayName',names{c});
    plot(ax3,t(1:end-1),uSteadyAll{c},'LineWidth',1.2,'DisplayName',names{c});
    semilogy(ax4,t(1:end-1),max(oneStepErrNormAll{c},1e-12), ...
        'LineWidth',1.2,'DisplayName',names{c});
end

title(ax1,'q_3 Tracking');
xlabel(ax1,'Time [s]');
ylabel(ax1,'q_3 [m]');
legend(ax1,'Location','best');

title(ax2,'Applied Control Input');
xlabel(ax2,'Time [s]');
ylabel(ax2,'u [N]');
legend(ax2,'Location','best');

title(ax3,'Optimized Steady Input');
xlabel(ax3,'Time [s]');
ylabel(ax3,'u_s [N]');
legend(ax3,'Location','best');

title(ax4,'One-Step Prediction Error');
xlabel(ax4,'Time [s]');
ylabel(ax4,'||e_{1-step}||_2');
legend(ax4,'Location','best');

title(tl,'DeePC Terminal Study');

figure('Color','w','Name','DeePC Terminal Study - Behavioral','Position',[120 80 1400 850]);

tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

ax1 = nexttile; hold on; grid on; box on;
ax2 = nexttile; hold on; grid on; box on;
ax3 = nexttile; hold on; grid on; box on;
ax4 = nexttile; hold on; grid on; box on;

for c = 1:nControllers

    semilogy(ax1,t(1:end-1),max(gNormAll{c},1e-12), ...
        'LineWidth',1.2,'DisplayName',names{c});

    semilogy(ax2,t(1:end-1),max(sigmaNormAll{c},1e-12), ...
        'LineWidth',1.2,'DisplayName',names{c});

    plot(ax3,t(1:end-1),1e3*solveTimeAll{c}, ...
        'LineWidth',1.2,'DisplayName',names{c});

    if any(isfinite(terminalStateNormAll{c}))
        semilogy(ax4,t(1:end-1),max(terminalStateNormAll{c},1e-12), ...
            'LineWidth',1.2,'DisplayName',names{c});
    end
end

title(ax1,'Behavioral Coefficient Norm');
xlabel(ax1,'Time [s]');
ylabel(ax1,'||g||_2');
legend(ax1,'Location','best');

title(ax2,'Past-Trajectory Slack Norm');
xlabel(ax2,'Time [s]');
ylabel(ax2,'||\sigma||_2');
legend(ax2,'Location','best');

title(ax3,'QP Solve Time');
xlabel(ax3,'Time [s]');
ylabel(ax3,'Solve time [ms]');
legend(ax3,'Location','best');

title(ax4,'Terminal State Metric');
xlabel(ax4,'Time [s]');
ylabel(ax4,'Terminal norm');
legend(ax4,'Location','best');

title(tl,'Terminal Formulation Diagnostics');

figure('Color','w','Name','DeePC Terminal Study - Nonlinearity','Position',[150 120 1300 600]);

hold on; grid on; box on;

for c = 1:nControllers

    x = xAll{c};

    dq23 = x(5,:)'-x(3,:)';
    Flinear23 = p.k23*dq23;
    Fcubic23 = p.k3*dq23.^3;
    nonlinearRatio = 100*abs(Fcubic23)./(abs(Flinear23)+1e-12);

    plot(t,nonlinearRatio,'LineWidth',1.2,'DisplayName',names{c});
end

xlabel('Time [s]');
ylabel('|F_{cubic}| / |F_{linear}| [%]');
title('Plant Nonlinearity Exposure');
legend('Location','best');

fprintf('\nMAX NONLINEAR EXPOSURE\n');
fprintf('------------------------------------------------\n');

for c = 1:nControllers
    fprintf('%-25s : %.2f %%\n',names{c},maxNonlinearAll(c));
end