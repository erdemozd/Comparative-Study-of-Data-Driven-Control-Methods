clear; clc; close all;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
resultFolder = fullfile(projectRoot,'results');

files = {
    'mpc_three_mass_diagnostics.mat'
    'deepc_three_mass_diagnostics.mat'
    'sdmpc_three_mass_diagnostics.mat'
    'hdeepc_three_mass_diagnostics.mat'
    'pzdeepc_4pole_three_mass_diagnostics.mat'};

names = {
    'MPC'
    'DeePC'
    'SD-MPC'
    'HDeePC'
    'PZ-DeePC 4-pole'};

%% ========================================================================
% Fixed controller colors
%
% The same controller uses the same color in every figure.
% ========================================================================

colors = {
    [0.0000 0.4470 0.7410]   % MPC             - blue
    [0.8500 0.3250 0.0980]   % DeePC           - orange
    [0.9290 0.6940 0.1250]   % SD-MPC          - yellow
    [0.4660 0.6740 0.1880]   % HDeePC-QP       - green
    [0.4940 0.1840 0.5560]}; % PZ-DeePC 4-pole - purple

data = cell(numel(files),1);
valid = false(numel(files),1);

for i = 1:numel(files)
    f = fullfile(resultFolder,files{i});

    if ~exist(f,'file')
        fprintf('%-18s : result not found\n',names{i});
        continue;
    end

    S = load(f);

    if isfield(S,'result')
        data{i} = S.result;
    else
        data{i} = S;
    end

    valid(i) = true;
end

%% ========================================================================
% Figure 1: main closed-loop comparison
% ========================================================================

figure('Color','w','Name','Controller Closed-Loop Comparison',...
    'Position',[80 60 1500 850],'WindowState','maximized');

tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

ax1 = nexttile; hold on; grid on; box on;
ax2 = nexttile; hold on; grid on; box on;
ax3 = nexttile; hold on; grid on; box on;
ax4 = nexttile; hold on; grid on; box on;

rDone = false;

for i = find(valid)'

    D = data{i};

    t = D.t(:);
    r = D.r(:);
    x = D.x;

    if size(x,1) ~= 6
        x = x';
    end

    q3 = x(5,:)';

    if ~rDone
        plot(ax1,t,r,'k--','LineWidth',1.8,'DisplayName','Reference');
        rDone = true;
    end

    plot(ax1,t,q3,...
        'Color',colors{i},...
        'LineWidth',1.3,...
        'DisplayName',names{i});

    if isfield(D,'u')
        plot(ax2,t,D.u(:),...
            'Color',colors{i},...
            'LineWidth',1.2,...
            'DisplayName',names{i});
    end

    if isfield(D,'uSteady')
        ts = t(1:min(numel(t),numel(D.uSteady)));

        plot(ax3,ts,D.uSteady(1:numel(ts)),...
            'Color',colors{i},...
            'LineWidth',1.2,...
            'DisplayName',names{i});
    end

    if isfield(D,'oneStepErr')
        te = t(1:min(numel(t)-1,numel(D.oneStepErr)));

        plot(ax4,te,D.oneStepErr(1:numel(te)),...
            'Color',colors{i},...
            'LineWidth',1.2,...
            'DisplayName',names{i});
    end
end

title(ax1,'q_3 Tracking');
xlabel(ax1,'Time [s]');
ylabel(ax1,'q_3 [m]');
legend(ax1,'Location','best');

title(ax2,'Applied Control Input');
xlabel(ax2,'Time [s]');
ylabel(ax2,'u [N]');
legend(ax2,'Location','best');

title(ax3,'Internally Optimized Input u_s');
xlabel(ax3,'Time [s]');
ylabel(ax3,'u_s [N]');
legend(ax3,'Location','best');

title(ax4,'One-Step State Prediction Error');
xlabel(ax4,'Time [s]');
ylabel(ax4,'||e_{1-step}||_2');
set(ax4,'YScale','log');
legend(ax4,'Location','best');

title(tl,'Three-Mass Controller Comparison');

%% ========================================================================
% Figure 2: computation + behavioral quantities
% ========================================================================

figure('Color','w','Name','Controller Diagnostic Comparison',...
    'Position',[100 80 1500 850], 'WindowState','maximized');

tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

ax1 = nexttile; hold on; grid on; box on;
ax2 = nexttile; hold on; grid on; box on;
ax3 = nexttile; hold on; grid on; box on;
ax4 = nexttile; hold on; grid on; box on;

for i = find(valid)'

    D = data{i};
    t = D.t(:);

    if isfield(D,'solveTime')
        ts = t(1:min(numel(t)-1,numel(D.solveTime)));

        plot(ax1,ts,1000*D.solveTime(1:numel(ts)),...
            'Color',colors{i},...
            'LineWidth',1.1,...
            'DisplayName',names{i});
    end

    if isfield(D,'gNorm')
        tg = t(1:min(numel(t)-1,numel(D.gNorm)));

        semilogy(ax2,tg,max(D.gNorm(1:numel(tg)),1e-12),...
            'Color',colors{i},...
            'LineWidth',1.1,...
            'DisplayName',names{i});
    end

    if isfield(D,'sigmaNorm')
        ts = t(1:min(numel(t)-1,numel(D.sigmaNorm)));

        semilogy(ax3,ts,max(D.sigmaNorm(1:numel(ts)),1e-12),...
            'Color',colors{i},...
            'LineWidth',1.1,...
            'DisplayName',names{i});
    end

    if isfield(D,'nonlinearRatio')
        tn = t(1:min(numel(t),numel(D.nonlinearRatio)));

        plot(ax4,tn,D.nonlinearRatio(1:numel(tn)),...
            'Color',colors{i},...
            'LineWidth',1.1,...
            'DisplayName',names{i});
    end
end

title(ax1,'QP Solve Time');
xlabel(ax1,'Time [s]');
ylabel(ax1,'Solve time [ms]');
legend(ax1,'Location','best');

title(ax2,'Behavioral Coefficient Norm');
xlabel(ax2,'Time [s]');
ylabel(ax2,'||g||_2');
legend(ax2,'Location','best');

title(ax3,'Past-Trajectory Slack Norm');
xlabel(ax3,'Time [s]');
ylabel(ax3,'||\sigma||_2');
legend(ax3,'Location','best');

title(ax4,'Nonlinear Spring Exposure');
xlabel(ax4,'Time [s]');
ylabel(ax4,'Cubic / linear [%]');
legend(ax4,'Location','best');

title(tl,'Optimization and Behavioral Diagnostics');

%% ========================================================================
% Console summary
% ========================================================================

fprintf('\n');
fprintf('THREE-MASS CONTROLLER COMPARISON\n');
fprintf('===============================================================================================\n');
fprintf('%-18s %9s %9s %9s %10s %10s %10s %10s\n',...
    'Controller','RMSE','max|u|','RMS u','solve ms','1-step','mean |g|','mean slack');
fprintf('-----------------------------------------------------------------------------------------------\n');

for i = find(valid)'

    D = data{i};

    t = D.t(:);
    r = D.r(:);
    x = D.x;

    if size(x,1) ~= 6
        x = x';
    end

    q3 = x(5,:)';

    n = min(numel(r),numel(q3));
    rmse = sqrt(mean((q3(1:n)-r(1:n)).^2));

    maxU = NaN;
    rmsU = NaN;
    meanSolve = NaN;
    meanErr = NaN;
    meanG = NaN;
    meanSigma = NaN;

    if isfield(D,'u')
        maxU = max(abs(D.u));
        rmsU = sqrt(mean(D.u.^2));
    end

    if isfield(D,'solveTime')
        meanSolve = 1000*mean(D.solveTime,'omitnan');
    end

    if isfield(D,'oneStepErr')
        meanErr = mean(D.oneStepErr,'omitnan');
    end

    if isfield(D,'gNorm')
        meanG = mean(D.gNorm,'omitnan');
    end

    if isfield(D,'sigmaNorm')
        meanSigma = mean(D.sigmaNorm,'omitnan');
    end

    fprintf('%-18s %9.4f %9.3f %9.3f %10.1f %10.3e %10.3e %10.3e\n',...
        names{i},rmse,maxU,rmsU,meanSolve,meanErr,meanG,meanSigma);
end

fprintf('===============================================================================================\n');

% %% ========================================================================
% % Figure 3: nonlinear excitation comparison
% % ========================================================================
% 
% figure('Color','w','Name','Controller Nonlinearity Comparison',...
%     'Position',[150 120 1300 600]);
% 
% hold on; grid on; box on;
% 
% for i = find(valid)'
% 
%     D = data{i};
% 
%     if ~isfield(D,'x')
%         continue;
%     end
% 
%     x = D.x;
% 
%     if size(x,1) ~= 6
%         x = x';
%     end
% 
%     dq23 = x(5,:) - x(3,:);
% 
%     if isfield(D,'nonlinearRatio')
%         ratio = D.nonlinearRatio(:)';
%     elseif isfield(D,'p') && isfield(D.p,'k3') && isfield(D.p,'k23')
%         ratio = 100*(D.p.k3/D.p.k23).*dq23.^2;
%     else
%         continue;
%     end
% 
%     n = min(numel(D.t),numel(ratio));
% 
%     plot(D.t(1:n),ratio(1:n),...
%         'Color',colors{i},...
%         'LineWidth',1.2,...
%         'DisplayName',names{i});
% end
% 
% xlabel('Time [s]');
% ylabel('|F_{cubic}| / |F_{linear}| [%]');
% title('Plant Nonlinearity Exposure');
% legend('Location','best');
% 
% %% ========================================================================
% % Figure 4: control-signal comparison
% % ========================================================================
% 
% figure('Color','w','Name','Controller Control-Signal Comparison',...
%     'Position',[120 80 1400 850]);
% 
% tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
% 
% ax1 = nexttile; hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
% ax2 = nexttile; hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
% ax3 = nexttile; hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
% 
% for i = find(valid)'
% 
%     D = data{i};
% 
%     if ~isfield(D,'t')
%         continue;
%     end
% 
%     t = D.t(:);
% 
%     if isfield(D,'u')
%         nu = min(numel(t),numel(D.u));
% 
%         plot(ax1,t(1:nu),D.u(1:nu),...
%             'Color',colors{i},...
%             'LineWidth',1.3,...
%             'DisplayName',names{i});
%     end
% 
%     if isfield(D,'uSteady')
%         ns = min(numel(t),numel(D.uSteady));
% 
%         plot(ax2,t(1:ns),D.uSteady(1:ns),...
%             'Color',colors{i},...
%             'LineWidth',1.3,...
%             'DisplayName',names{i});
%     end
% 
%     if isfield(D,'u') && isfield(D,'uSteady')
%         nd = min([numel(t),numel(D.u),numel(D.uSteady)]);
% 
%         du = D.u(1:nd) - D.uSteady(1:nd);
% 
%         plot(ax3,t(1:nd),du,...
%             'Color',colors{i},...
%             'LineWidth',1.3,...
%             'DisplayName',names{i});
%     end
% end
% 
% title(ax1,'Applied control input');
% xlabel(ax1,'Time [s]');
% ylabel(ax1,'u [N]');
% legend(ax1,'Location','best');
% 
% title(ax2,'Optimized steady input');
% xlabel(ax2,'Time [s]');
% ylabel(ax2,'u_s [N]');
% legend(ax2,'Location','best');
% 
% title(ax3,'Transient control component');
% xlabel(ax3,'Time [s]');
% ylabel(ax3,'u - u_s [N]');
% legend(ax3,'Location','best');
% 
% title(tl,'Control-Signal Diagnostics');