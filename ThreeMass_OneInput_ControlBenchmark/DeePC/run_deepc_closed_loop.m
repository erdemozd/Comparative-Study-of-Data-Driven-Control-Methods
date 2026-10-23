clear; clc; close all;

p = three_mass_params();
s = benchmark_settings(p);

projectRoot = fileparts(fileparts(mfilename('fullpath')));

if isempty(projectRoot)
    projectRoot = pwd;
end

resultsFolder = fullfile(projectRoot,'results');

if ~exist(resultsFolder,'dir')
    mkdir(resultsFolder);
end

dataFile = fullfile(projectRoot,'data','deepc_training_data.mat');

if ~exist(dataFile,'file')
    error('Run DeePC/generate_deepc_data.m first.');
end

load(dataFile,'deepcData');

ctrl = deepc_setup(p,deepcData);

t = (0:p.Ts:s.Tend)';
Ns = numel(t);
r = build_reference(t);

x = zeros(6,Ns);
u = zeros(Ns,1);

uBuffer = zeros(1,ctrl.Tini);
yBuffer = zeros(6,ctrl.Tini);

solveTime = nan(Ns-1,1);
Jlog = nan(Ns-1,1);
sigmaNorm = nan(Ns-1,1);
gNorm = nan(Ns-1,1);
uSteady = nan(Ns-1,1);
fail = false(Ns-1,1);

predCurrent = nan(6,Ns-1);
predNext = nan(6,Ns-1);

currentStateErr = nan(6,Ns-1);
oneStepErr = nan(6,Ns-1);

for k = 1:Ns-1
    rPreview = repmat(r(k),ctrl.N,1);
    rDotPreview = zeros(ctrl.N,1);

    [uc,info] = deepc_control(uBuffer(:),yBuffer(:),rPreview,rDotPreview,p,ctrl);

    if info.success
        uk = uc;

        Jlog(k) = info.J;
        sigmaNorm(k) = info.sigmaNorm;

        if isfield(info,'g') && ~isempty(info.g)
            gNorm(k) = norm(info.g,2);
        end

        if isfield(info,'uSteady')
            uSteady(k) = info.uSteady;
        end

        if isfield(info,'xPred') && ~isempty(info.xPred)
            if size(info.xPred,2)>=1
                predCurrent(:,k) = info.xPred(:,1);
            end

            if size(info.xPred,2)>=2
                predNext(:,k) = info.xPred(:,2);
            end
        end
    else
        fail(k) = true;
        uk = 0;

        if k>1
            uk = u(k-1);
        end
    end

    uk = min(max(uk,-ctrl.uMax),ctrl.uMax);
    u(k) = uk;
    solveTime(k) = info.solveTime;

    if all(isfinite(predCurrent(:,k)))
        currentStateErr(:,k) = x(:,k)-predCurrent(:,k);
    end

    uBuffer = [uBuffer(:,2:end),uk];
    yBuffer = [yBuffer(:,2:end),x(:,k)];

    x(:,k+1) = rk4_step(x(:,k),uk,p.Ts,p);

    if all(isfinite(predNext(:,k)))
        oneStepErr(:,k) = x(:,k+1)-predNext(:,k);
    end
end

u(end) = u(end-1);

%% Performance
q3 = x(5,:)';
rmse = sqrt(mean((q3-r).^2));

dq23 = x(5,:)'-x(3,:)';
Flinear23 = p.k23*dq23;
Fcubic23 = p.k3*dq23.^3;
nonlinearRatio = 100*abs(Fcubic23)./(abs(Flinear23)+1e-12);

currentStateErrNorm = vecnorm(currentStateErr,2,1)';
oneStepErrNorm = vecnorm(oneStepErr,2,1)';

fprintf('\nDEEPC DIAGNOSTIC SUMMARY\n');
fprintf('q3 RMSE                  : %.6f m\n',rmse);
fprintf('QP failures              : %d\n',sum(fail));
fprintf('Mean solve time          : %.3f ms\n',1e3*mean(solveTime,'omitnan'));
fprintf('Maximum solve time       : %.3f ms\n',1e3*max(solveTime,[],'omitnan'));
fprintf('Deadline misses (> Ts)   : %d / %d\n',sum(solveTime>p.Ts),Ns-1);
fprintf('Mean slack norm          : %.3e\n',mean(sigmaNorm,'omitnan'));
fprintf('Mean g norm              : %.3e\n',mean(gNorm,'omitnan'));
fprintf('Mean current-state error : %.3e\n',mean(currentStateErrNorm,'omitnan'));
fprintf('Mean one-step error      : %.3e\n',mean(oneStepErrNorm,'omitnan'));
fprintf('Max nonlinear ratio      : %.2f %%\n\n',max(nonlinearRatio));

%% Closed-loop diagnostics
figure('Color','w','Name','DeePC - Closed-Loop Diagnostics');

tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t,r,'k--','LineWidth',1.4);
hold on;
plot(t,q3,'LineWidth',1.5);
grid on;
ylabel('q_3 [m]');
title(sprintf('Tracking, RMSE = %.4f m',rmse));
legend('r','q_3','Location','best');

nexttile;
plot(t,x(1,:)','LineWidth',1.1);
hold on;
plot(t,x(3,:)','LineWidth',1.1);
plot(t,x(5,:)','LineWidth',1.3);
grid on;
ylabel('Position [m]');
title('All positions');
legend('q_1','q_2','q_3','Location','best');

nexttile;
plot(t,x(2,:)','LineWidth',1.1);
hold on;
plot(t,x(4,:)','LineWidth',1.1);
plot(t,x(6,:)','LineWidth',1.1);
grid on;
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Velocities');
legend('v_1','v_2','v_3','Location','best');

nexttile;
plot(t,u,'LineWidth',1.3);
hold on;
plot(t(1:end-1),uSteady,'LineWidth',1.1);
plot(t,p.k1*r,'k--','LineWidth',1.0);
yline(ctrl.uMax,'--');
yline(-ctrl.uMax,'--');
grid on;
xlabel('Time [s]');
ylabel('Force [N]');
title('Applied / optimized steady input');
legend('u','u_s','k_1r diagnostic','Location','best');

title(tl,'Standard DeePC');

%% Behavioral diagnostics
figure('Color','w','Name','DeePC - Behavioral Diagnostics');

tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile;
semilogy(t(1:end-1),max(sigmaNorm,1e-12),'LineWidth',1.2);
grid on;
ylabel('||\sigma||_2');
title('Past-trajectory slack norm');

nexttile;
semilogy(t(1:end-1),max(gNorm,1e-12),'LineWidth',1.2);
grid on;
ylabel('||g||_2');
title('Behavioral coefficient norm');

nexttile;
plot(t(2:end),x(5,2:end)','LineWidth',1.3);
hold on;
plot(t(2:end),predNext(5,:)','--','LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('q_3 [m]');
title('One-step prediction');
legend('Actual','Predicted','Location','best');

nexttile;
semilogy(t(1:end-1),max(oneStepErrNorm,1e-12),'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('||e_{1-step}||_2');
title('One-step state prediction error');

title(tl,'DeePC Behavioral Quantities');

%% Optimization load
figure('Color','w','Name','DeePC - Optimization Load');

tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t(1:end-1),1e3*solveTime,'LineWidth',1.2);
hold on;
yline(1e3*p.Ts,'k--');
grid on;
ylabel('Solve time [ms]');
title('QP solve time');

nexttile;
plot(t(1:end-1),100*solveTime/p.Ts,'LineWidth',1.2);
hold on;
yline(100,'k--');
grid on;
ylabel('Load [% of T_s]');
title('Real-time computational load');

nexttile;
plot(t(1:end-1),Jlog,'LineWidth',1.2);
grid on;
xlabel('Time [s]');
ylabel('J^*');
title('Optimal objective value');

nexttile;
stairs(t(1:end-1),double(fail),'LineWidth',1.2);
ylim([-0.1 1.1]);
grid on;
xlabel('Time [s]');
ylabel('Failure');
title('Solver failures');

title(tl,'DeePC Optimization Diagnostics');

%% Plant nonlinearity
figure('Color','w','Name','DeePC - Plant Nonlinearity');

tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t,dq23,'LineWidth',1.3);
grid on;
ylabel('q_3-q_2 [m]');
title('Nonlinear spring deformation');

nexttile;
plot(t,Flinear23,'LineWidth',1.1);
hold on;
plot(t,Fcubic23,'LineWidth',1.1);

yyaxis right;
plot(t,nonlinearRatio,'--','LineWidth',1.1);
ylabel('Cubic/linear [%]');

yyaxis left;
ylabel('Force [N]');
grid on;
xlabel('Time [s]');
legend('Linear force','Cubic force','Nonlinear ratio','Location','best');
title('Nonlinear spring exposure');

title(tl,'True Plant Nonlinearity');

%% Save
result = struct();

result.t = t;
result.x = x;
result.u = u;
result.r = r;
result.rmse = rmse;

result.solveTime = solveTime;
result.J = Jlog;
result.sigmaNorm = sigmaNorm;
result.gNorm = gNorm;
result.uSteady = uSteady;
result.fail = fail;

result.predCurrent = predCurrent;
result.predNext = predNext;

result.currentStateErr = currentStateErr;
result.currentStateErrNorm = currentStateErrNorm;

result.oneStepErr = oneStepErr;
result.oneStepErrNorm = oneStepErrNorm;

result.dq23 = dq23;
result.Flinear23 = Flinear23;
result.Fcubic23 = Fcubic23;
result.nonlinearRatio = nonlinearRatio;

result.ctrl = ctrl;
result.p = p;

save(fullfile(resultsFolder,'deepc_three_mass_diagnostics.mat'),'result');
