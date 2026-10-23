clear; clc; close all;

p = three_mass_params(); s = benchmark_settings(p);
projectRoot = fileparts(fileparts(mfilename('fullpath'))); if isempty(projectRoot), projectRoot=pwd; end
resultsFolder=fullfile(projectRoot,'results'); if ~exist(resultsFolder,'dir'), mkdir(resultsFolder); end
dataFile=fullfile(projectRoot,'data','sdmpc_residual_data.mat'); if ~exist(dataFile,'file'), error('Run SD-MPC/generate_sdmpc_data.m first.'); end
load(dataFile,'sdmpcData'); ctrl=sdmpc_setup(p,sdmpcData);

t=(0:p.Ts:s.Tend)'; Ns=numel(t); r=build_reference(t);
x=zeros(6,Ns); xM=zeros(6,Ns); u=zeros(Ns,1);
uBuffer=zeros(1,ctrl.Tini); resBuffer=zeros(6,ctrl.Tini);
solveTime=nan(Ns-1,1); Jlog=nan(Ns-1,1); sigmaNorm=nan(Ns-1,1); gNorm=nan(Ns-1,1); uSteady=nan(Ns-1,1); fail=false(Ns-1,1);
predNext=nan(6,Ns-1); oneStepErr=nan(6,Ns-1); nomPredNext=nan(6,Ns-1); resPredNext=nan(6,Ns-1); corrPredNext=nan(6,Ns-1);

for k=1:Ns-1
    rPreview=repmat(r(k),ctrl.N,1); rDotPreview=zeros(ctrl.N,1);
    [uc,info]=sdmpc_control(uBuffer(:),resBuffer(:),xM(:,k),rPreview,rDotPreview,p,ctrl);
    if info.success
        uk=uc; Jlog(k)=info.J; sigmaNorm(k)=info.sigmaNorm;
        if isfield(info,'g') && ~isempty(info.g)
            gNorm(k)=norm(info.g,2);
            Upred=ctrl.Uf*info.g;
            xNomPred=reshape(ctrl.Sx*xM(:,k)+ctrl.Su*Upred,6,ctrl.N);
            xResPred=reshape(ctrl.Rf*info.g,6,ctrl.N);
            if ctrl.N>=2
                nomPredNext(:,k)=xNomPred(:,2); resPredNext(:,k)=xResPred(:,2); corrPredNext(:,k)=nomPredNext(:,k)+resPredNext(:,k);
            end
        end
        if isfield(info,'uSteady'), uSteady(k)=info.uSteady; end
        if isfield(info,'xPred') && ~isempty(info.xPred) && size(info.xPred,2)>=2, predNext(:,k)=info.xPred(:,2); end
    else
        fail(k)=true; uk=0; if k>1, uk=u(k-1); end
    end
    uk=min(max(uk,-ctrl.uMax),ctrl.uMax); u(k)=uk; solveTime(k)=info.solveTime;
    currentResidual=x(:,k)-xM(:,k);
    uBuffer=[uBuffer(:,2:end),uk]; resBuffer=[resBuffer(:,2:end),currentResidual];
    x(:,k+1)=rk4_step(x(:,k),uk,p.Ts,p);
    xM(:,k+1)=ctrl.Ad*xM(:,k)+ctrl.Bd*uk;
    if all(isfinite(predNext(:,k))), oneStepErr(:,k)=x(:,k+1)-predNext(:,k); end
end
u(end)=u(end-1);
%%
residual=x-xM; residualNorm=vecnorm(residual,2,1)'; q3=x(5,:)'; rmse=sqrt(mean((q3-r).^2));
dq23=x(5,:)'-x(3,:)'; Flinear23=p.k23*dq23; Fcubic23=p.k3*dq23.^3; nonlinearRatio=100*abs(Fcubic23)./(abs(Flinear23)+1e-12);

fprintf('\nSD-MPC DIAGNOSTIC SUMMARY\n');
fprintf('q3 RMSE                  : %.6f m\n',rmse); fprintf('QP failures              : %d\n',sum(fail));
fprintf('Mean solve time          : %.3f ms\n',1e3*mean(solveTime,'omitnan')); fprintf('Deadline misses (> Ts)   : %d / %d\n',sum(solveTime>p.Ts),Ns-1);
fprintf('Mean measured residual   : %.3e\n',mean(residualNorm)); fprintf('Mean slack norm          : %.3e\n',mean(sigmaNorm,'omitnan'));
fprintf('Max nonlinear ratio      : %.2f %%\n\n',max(nonlinearRatio));

figure('Color','w','Name','SD-MPC - Closed-Loop Diagnostics'); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(t,r,'k--','LineWidth',1.4); hold on; plot(t,q3,'LineWidth',1.5); grid on; ylabel('q_3 [m]'); title(sprintf('Tracking, RMSE = %.4f m',rmse)); legend('r','q_3','Location','best');
nexttile; plot(t,x(1,:)'); hold on; plot(t,x(3,:)'); plot(t,x(5,:)'); grid on; ylabel('Position [m]'); title('Actual positions'); legend('q_1','q_2','q_3','Location','best');
nexttile; plot(t,x(2,:)'); hold on; plot(t,x(4,:)'); plot(t,x(6,:)'); grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]'); title('Actual velocities'); legend('v_1','v_2','v_3','Location','best');
nexttile; plot(t,u,'LineWidth',1.3); hold on; plot(t(1:end-1),uSteady,'LineWidth',1.1); plot(t,p.k1*r,'k--'); grid on; xlabel('Time [s]'); ylabel('Force [N]'); title('Input and optimized u_s'); legend('u','u_s','k_1r diagnostic','Location','best'); title(tl,'SD-MPC');

figure('Color','w','Name','SD-MPC - Residual Diagnostics'); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(t,x(5,:)','LineWidth',1.3); hold on; plot(t,xM(5,:)','--','LineWidth',1.2); grid on; ylabel('q_3 [m]'); title('True plant vs nominal model'); legend('True q_3','Nominal q_{3,M}','Location','best');
nexttile; plot(t,residual(1,:)'); hold on; plot(t,residual(3,:)'); plot(t,residual(5,:)'); grid on; ylabel('Position residual [m]'); title('Measured model residuals'); legend('q_1 residual','q_2 residual','q_3 residual','Location','best');
nexttile; semilogy(t,max(residualNorm,1e-12),'LineWidth',1.2); grid on; xlabel('Time [s]'); ylabel('||x-x_M||_2'); title('Residual-state norm');
nexttile; plot(t(2:end),x(5,2:end)','LineWidth',1.3); hold on; plot(t(2:end),nomPredNext(5,:)','--','LineWidth',1.1); plot(t(2:end),corrPredNext(5,:)','-.','LineWidth',1.2); grid on; xlabel('Time [s]'); ylabel('q_3 [m]'); title('Residual correction in one-step prediction'); legend('Actual','Nominal prediction','Nominal + residual prediction','Location','best'); title(tl,'SD-MPC Residual Contribution');

figure('Color','w','Name','SD-MPC - Optimization Diagnostics'); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; semilogy(t(1:end-1),max(sigmaNorm,1e-12)); grid on; ylabel('||\sigma||_2'); title('Residual-past slack');
nexttile; semilogy(t(1:end-1),max(gNorm,1e-12)); grid on; ylabel('||g||_2'); title('Behavioral coefficient norm');
nexttile; plot(t(1:end-1),1e3*solveTime); hold on; yline(1e3*p.Ts,'k--'); grid on; xlabel('Time [s]'); ylabel('Solve time [ms]'); title('QP computation');
nexttile; plot(t(1:end-1),Jlog); grid on; xlabel('Time [s]'); ylabel('J^*'); title('Optimal objective'); title(tl,'SD-MPC Optimization');

figure('Color','w','Name','SD-MPC - Prediction Errors'); tl=tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile; semilogy(t(1:end-1),max(vecnorm(oneStepErr,2,1)',1e-12)); grid on; ylabel('||e_{1-step}||_2'); title('Corrected one-step prediction error');
nexttile; plot(t(2:end),resPredNext(5,:)','LineWidth',1.2); hold on; plot(t(2:end),residual(5,2:end)','--','LineWidth',1.2); grid on; xlabel('Time [s]'); ylabel('q_3 residual [m]'); title('Predicted residual contribution vs measured residual'); legend('Predicted residual','Measured residual','Location','best'); title(tl,'SD-MPC Prediction Quality');

result=struct(); result.t=t; result.x=x; result.xM=xM; result.residual=residual; result.residualNorm=residualNorm; result.u=u; result.r=r; result.rmse=rmse;
result.solveTime=solveTime; result.J=Jlog; result.sigmaNorm=sigmaNorm; result.gNorm=gNorm; result.uSteady=uSteady; result.fail=fail;
result.predNext=predNext; result.oneStepErr=oneStepErr; result.nomPredNext=nomPredNext; result.resPredNext=resPredNext; result.corrPredNext=corrPredNext;
result.dq23=dq23; result.Flinear23=Flinear23; result.Fcubic23=Fcubic23; result.nonlinearRatio=nonlinearRatio; result.ctrl=ctrl; result.p=p;
save(fullfile(resultsFolder,'sdmpc_three_mass_diagnostics.mat'),'result');
