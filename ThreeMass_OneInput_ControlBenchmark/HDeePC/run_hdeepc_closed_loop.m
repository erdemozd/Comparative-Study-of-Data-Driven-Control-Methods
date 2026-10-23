clear; clc; close all;

% Output sensor noise: offline predictor data and online feedback are independent.
% Applied training inputs are retained in both offline modes.
offlineDataMode = 'noisy';  % 'clean' or 'noisy'
measurementMode = 'noisy';  % 'clean' or 'noisy'
measurementNoiseSeed = 42;

p=three_mass_params(); s=benchmark_settings(p);
projectRoot=fileparts(fileparts(mfilename('fullpath'))); if isempty(projectRoot), projectRoot=pwd; end
resultsFolder=fullfile(projectRoot,'results'); if ~exist(resultsFolder,'dir'), mkdir(resultsFolder); end
dataFile=fullfile(projectRoot,'data','deepc_training_data.mat'); if ~exist(dataFile,'file'), error('Run DeePC/generate_deepc_data.m first.'); end
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
ctrl=hdeepc_setup(p,deepcData);

t=(0:p.Ts:s.Tend)'; Ns=numel(t); r=build_reference(t); x=zeros(6,Ns); u=zeros(Ns,1);
uBuffer=zeros(1,ctrl.Tini); yuBuffer=zeros(4,ctrl.Tini);
solveTime=nan(Ns-1,1); Jlog=nan(Ns-1,1); sigmaNorm=nan(Ns-1,1); gNorm=nan(Ns-1,1); uSteady=nan(Ns-1,1); fail=false(Ns-1,1); predNext=nan(6,Ns-1); oneStepErr=nan(6,Ns-1);

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

for k=1:Ns-1
    yMeasured(:,k) = x(:,k) + measurementNoise(:,k);
    rPreview=repmat(r(k),ctrl.N,1); rDotPreview=zeros(ctrl.N,1); xK0=yMeasured(1:2,k);
    [uc,info]=hdeepc_control(uBuffer(:),yuBuffer(:),xK0,rPreview,rDotPreview,p,ctrl);
    if info.success
        uk=uc; Jlog(k)=info.J; sigmaNorm(k)=info.sigmaNorm;
        if isfield(info,'g') && ~isempty(info.g), gNorm(k)=norm(info.g,2); end
        if isfield(info,'uSteady'), uSteady(k)=info.uSteady; end
        if isfield(info,'xPred') && ~isempty(info.xPred) && size(info.xPred,2)>=2, predNext(:,k)=info.xPred(:,2); end
    else
        fail(k)=true; uk=0; if k>1, uk=u(k-1); end
    end
    uk=min(max(uk,-ctrl.uMax),ctrl.uMax); u(k)=uk; solveTime(k)=info.solveTime;
    uBuffer=[uBuffer(:,2:end),uk]; yuBuffer=[yuBuffer(:,2:end),yMeasured(3:6,k)];
    x(:,k+1)=rk4_step(x(:,k),uk,p.Ts,p);
    if all(isfinite(predNext(:,k))), oneStepErr(:,k)=x(:,k+1)-predNext(:,k); end
end
u(end)=u(end-1);
yMeasured(:,end) = x(:,end) + measurementNoise(:,end);
q3=x(5,:)'; rmse=sqrt(mean((q3-r).^2)); knownErr=vecnorm(oneStepErr(1:2,:),2,1)'; unknownErr=vecnorm(oneStepErr(3:6,:),2,1)';
dq23=x(5,:)'-x(3,:)'; Flinear23=p.k23*dq23; Fcubic23=p.k3*dq23.^3; nonlinearRatio=100*abs(Fcubic23)./(abs(Flinear23)+1e-12);

fprintf('\nHDeePC-QP DIAGNOSTIC SUMMARY\n'); fprintf('q3 RMSE: %.6f m\n',rmse); fprintf('Failures: %d\n',sum(fail)); fprintf('Mean solve time: %.3f ms\n',1e3*mean(solveTime,'omitnan')); fprintf('Deadline misses: %d / %d\n\n',sum(solveTime>p.Ts),Ns-1);

figure('Color','w','Name','HDeePC - Closed-Loop Diagnostics'); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(t,r,'k--','LineWidth',1.4); hold on; plot(t,q3,'LineWidth',1.5); grid on; ylabel('q_3 [m]'); title(sprintf('Tracking, RMSE = %.4f m',rmse)); legend('r','q_3');
nexttile; plot(t,x(1,:)'); hold on; plot(t,x(3,:)'); plot(t,x(5,:)'); grid on; ylabel('Position [m]'); title('Positions'); legend('q_1','q_2','q_3');
nexttile; plot(t,x(2,:)'); hold on; plot(t,x(4,:)'); plot(t,x(6,:)'); grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]'); title('Velocities'); legend('v_1','v_2','v_3');
nexttile; plot(t,u,'LineWidth',1.3); hold on; plot(t(1:end-1),uSteady); plot(t,p.k1*r,'k--'); grid on; xlabel('Time [s]'); ylabel('Force [N]'); title('Input / optimized u_s'); legend('u','u_s','k_1r diagnostic'); title(tl,'HDeePC-QP');

figure('Color','w','Name','HDeePC - Hybrid Prediction Diagnostics'); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; semilogy(t(1:end-1),max(sigmaNorm,1e-12)); grid on; ylabel('||\sigma||_2'); title('Unknown-block past slack');
nexttile; semilogy(t(1:end-1),max(gNorm,1e-12)); grid on; ylabel('||g||_2'); title('Behavioral coefficient norm');
nexttile; semilogy(t(1:end-1),max(knownErr,1e-12)); hold on; semilogy(t(1:end-1),max(unknownErr,1e-12)); grid on; xlabel('Time [s]'); ylabel('One-step error norm'); title('Known vs data-driven block prediction'); legend('Known x_\kappa=[q_1,v_1]','Unknown y_u=[q_2,v_2,q_3,v_3]');
nexttile; plot(t(2:end),x(5,2:end)'); hold on; plot(t(2:end),predNext(5,:)','--'); grid on; xlabel('Time [s]'); ylabel('q_3 [m]'); title('One-step q_3 prediction'); legend('Actual','Predicted'); title(tl,'HDeePC Hybrid Structure');

figure('Color','w','Name','HDeePC - Optimization Load'); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(t(1:end-1),1e3*solveTime); hold on; yline(1e3*p.Ts,'k--'); grid on; ylabel('Solve time [ms]'); title('QP solve time');
nexttile; plot(t(1:end-1),100*solveTime/p.Ts); hold on; yline(100,'k--'); grid on; ylabel('Load [% of T_s]'); title('Real-time load');
nexttile; plot(t(1:end-1),Jlog); grid on; xlabel('Time [s]'); ylabel('J^*'); title('Objective');
nexttile; stairs(t(1:end-1),double(fail)); ylim([-0.1 1.1]); grid on; xlabel('Time [s]'); ylabel('Failure'); title('Solver failures'); title(tl,'HDeePC Optimization');

result=struct(); result.t=t; result.x=x; result.u=u; result.r=r; result.rmse=rmse; result.solveTime=solveTime; result.J=Jlog; result.sigmaNorm=sigmaNorm; result.gNorm=gNorm; result.uSteady=uSteady; result.fail=fail; result.predNext=predNext; result.oneStepErr=oneStepErr; result.knownPredictionError=knownErr; result.unknownPredictionError=unknownErr; result.dq23=dq23; result.Flinear23=Flinear23; result.Fcubic23=Fcubic23; result.nonlinearRatio=nonlinearRatio; result.ctrl=ctrl; result.p=p;
result.offlineDataMode=offlineDataMode; result.measurementMode=measurementMode;
result.measurementNoiseSeed=measurementNoiseSeed; result.sigmaY=sigmaY;
result.measurementNoise=measurementNoise; result.yMeasured=yMeasured;
save(fullfile(resultsFolder,'hdeepc_three_mass_diagnostics.mat'),'result');
