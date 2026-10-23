    clear; clc; close all;

p=three_mass_params(); s=benchmark_settings(p);
projectRoot=fileparts(fileparts(mfilename('fullpath'))); if isempty(projectRoot), projectRoot=pwd; end
resultsFolder=fullfile(projectRoot,'results'); if ~exist(resultsFolder,'dir'), mkdir(resultsFolder); end
dataFile=fullfile(projectRoot,'data','deepc_training_data.mat'); if ~exist(dataFile,'file'), error('Run DeePC/generate_deepc_data.m first.'); end
load(dataFile,'deepcData');
ctrl=pzdeepc_setup_4pole(p,deepcData);

t=(0:p.Ts:s.Tend)'; Ns=numel(t); r=build_reference(t); x=zeros(6,Ns); u=zeros(Ns,1);
uBuffer=zeros(1,ctrl.Tini); yBuffer=zeros(6,ctrl.Tini);
solveTime=nan(Ns-1,1); Jlog=nan(Ns-1,1); sigmaNorm=nan(Ns-1,1); gNorm=nan(Ns-1,1); uSteady=nan(Ns-1,1); fail=false(Ns-1,1); predNext=nan(6,Ns-1); oneStepErr=nan(6,Ns-1);

for k=1:Ns-1
    rPreview=repmat(r(k),ctrl.N,1); rDotPreview=zeros(ctrl.N,1);
    [uc,info]=pzdeepc_control(uBuffer(:),yBuffer(:),rPreview,rDotPreview,p,ctrl);
    if info.success
        uk=uc; Jlog(k)=info.J; sigmaNorm(k)=info.sigmaNorm;
        if isfield(info,'g') && ~isempty(info.g), gNorm(k)=norm(info.g,2); end
        if isfield(info,'uSteady'), uSteady(k)=info.uSteady; end
        if isfield(info,'xPred') && ~isempty(info.xPred) && size(info.xPred,2)>=2, predNext(:,k)=info.xPred(:,2); end
    else
        fail(k)=true; uk=0; if k>1, uk=u(k-1); end
    end
    uk=min(max(uk,-ctrl.uMax),ctrl.uMax); u(k)=uk; solveTime(k)=info.solveTime;
    uBuffer=[uBuffer(:,2:end),uk]; yBuffer=[yBuffer(:,2:end),x(:,k)];
    x(:,k+1)=rk4_step(x(:,k),uk,p.Ts,p);
    if all(isfinite(predNext(:,k))), oneStepErr(:,k)=x(:,k+1)-predNext(:,k); end
end
u(end)=u(end-1);

q3=x(5,:)'; rmse=sqrt(mean((q3-r).^2));
dq23=x(5,:)'-x(3,:)'; Flinear23=p.k23*dq23; Fcubic23=p.k3*dq23.^3; nonlinearRatio=100*abs(Fcubic23)./(abs(Flinear23)+1e-12);

sp=ctrl.prior.sp;
yPrimeVec=pz_filter_past_output(x(:),ctrl.prior.polyCoeff,6);
yPrime=reshape(yPrimeVec,6,Ns-sp);
tPrime=t(sp+1:end);
w=linspace(0,pi,1000); zUnit=exp(1j*w); Hfilter=polyval(ctrl.prior.polyCoeff,zUnit);

fprintf('\n4-Pole PZ-DeePC DIAGNOSTIC SUMMARY\n');
fprintf('q3 RMSE                  : %.6f m\n',rmse); fprintf('QP failures              : %d\n',sum(fail));
fprintf('Mean solve time          : %.3f ms\n',1e3*mean(solveTime,'omitnan')); fprintf('Deadline misses (> Ts)   : %d / %d\n',sum(solveTime>p.Ts),Ns-1);
fprintf('Mean slack norm          : %.3e\n',mean(sigmaNorm,'omitnan')); fprintf('Mean g norm              : %.3e\n',mean(gNorm,'omitnan'));
fprintf('Known poles:\n'); disp(ctrl.prior.knownPoles); fprintf('Filter polynomial coefficients:\n'); disp(ctrl.prior.polyCoeff);

figure('Color','w','Name','4-Pole PZ-DeePC - Closed-Loop Diagnostics'); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(t,r,'k--','LineWidth',1.4); hold on; plot(t,q3,'LineWidth',1.5); grid on; ylabel('q_3 [m]'); title(sprintf('Tracking, RMSE = %.4f m',rmse)); legend('r','q_3');
nexttile; plot(t,x(1,:)'); hold on; plot(t,x(3,:)'); plot(t,x(5,:)'); grid on; ylabel('Position [m]'); title('Positions'); legend('q_1','q_2','q_3');
nexttile; plot(t,x(2,:)'); hold on; plot(t,x(4,:)'); plot(t,x(6,:)'); grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]'); title('Velocities'); legend('v_1','v_2','v_3');
nexttile; plot(t,u,'LineWidth',1.3); hold on; plot(t(1:end-1),uSteady); plot(t,p.k1*r,'k--'); grid on; xlabel('Time [s]'); ylabel('Force [N]'); title('Input / optimized u_s'); legend('u','u_s','k_1r diagnostic'); title(tl,'4-Pole PZ-DeePC');

figure('Color','w','Name','4-Pole PZ-DeePC - Behavioral Diagnostics'); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; semilogy(t(1:end-1),max(sigmaNorm,1e-12)); grid on; ylabel('||\sigma||_2'); title('Filtered-past slack norm');
nexttile; semilogy(t(1:end-1),max(gNorm,1e-12)); grid on; ylabel('||g||_2'); title('Behavioral coefficient norm');
nexttile; plot(t(2:end),x(5,2:end)'); hold on; plot(t(2:end),predNext(5,:)','--'); grid on; xlabel('Time [s]'); ylabel('q_3 [m]'); title('One-step prediction'); legend('Actual','Predicted');
nexttile; semilogy(t(1:end-1),max(vecnorm(oneStepErr,2,1)',1e-12)); grid on; xlabel('Time [s]'); ylabel('||e_{1-step}||_2'); title('One-step state prediction error'); title(tl,'4-Pole PZ-DeePC Behavioral Quantities');

figure('Color','w','Name','4-Pole PZ-DeePC - Pole Prior and Filter'); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; th=linspace(0,2*pi,400); plot(cos(th),sin(th),'k--'); hold on; scatter(real(ctrl.prior.allPoles),imag(ctrl.prior.allPoles),45,'o'); scatter(real(ctrl.prior.knownPoles),imag(ctrl.prior.knownPoles),80,'x','LineWidth',1.8); axis equal; grid on; xlabel('Re(z)'); ylabel('Im(z)'); title('Nominal poles and selected prior'); legend('Unit circle','All nominal poles','Known poles','Location','best');
nexttile; plot(t,x(5,:)','LineWidth',1.2); hold on; plot(tPrime,yPrime(5,:)','LineWidth',1.2); grid on; xlabel('Time [s]'); ylabel('Signal'); title('Raw and pole-filtered q_3'); legend('q_3','filtered q_3','Location','best');
nexttile; plot(w/pi,abs(Hfilter),'LineWidth',1.3); grid on; xlabel('\omega/\pi'); ylabel('|P(e^{j\omega})|'); title('Pole-polynomial filter magnitude');
nexttile; stem(0:numel(ctrl.prior.polyCoeff)-1,ctrl.prior.polyCoeff,'filled'); grid on; xlabel('Coefficient index'); ylabel('Coefficient'); title('P(z) coefficients'); title(tl,'4-Pole PZ-DeePC Prior-Knowledge Diagnostics');

figure('Color','w','Name','4-Pole PZ-DeePC - Optimization Load'); tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(t(1:end-1),1e3*solveTime); hold on; yline(1e3*p.Ts,'k--'); grid on; ylabel('Solve time [ms]'); title('QP solve time');
nexttile; plot(t(1:end-1),100*solveTime/p.Ts); hold on; yline(100,'k--'); grid on; ylabel('Load [% of T_s]'); title('Real-time load');
nexttile; plot(t(1:end-1),Jlog); grid on; xlabel('Time [s]'); ylabel('J^*'); title('Optimal objective');
nexttile; stairs(t(1:end-1),double(fail)); ylim([-0.1 1.1]); grid on; xlabel('Time [s]'); ylabel('Failure'); title('Solver failures'); title(tl,'4-Pole PZ-DeePC Optimization');

result=struct(); result.t=t; result.x=x; result.u=u; result.r=r; result.rmse=rmse; result.solveTime=solveTime; result.J=Jlog; result.sigmaNorm=sigmaNorm; result.gNorm=gNorm; result.uSteady=uSteady; result.fail=fail; result.predNext=predNext; result.oneStepErr=oneStepErr; result.filteredTime=tPrime; result.filteredState=yPrime; result.filterFrequency=w; result.filterResponse=Hfilter; result.dq23=dq23; result.Flinear23=Flinear23; result.Fcubic23=Fcubic23; result.nonlinearRatio=nonlinearRatio; result.ctrl=ctrl; result.p=p;
save(fullfile(resultsFolder,'pzdeepc_4pole_three_mass_diagnostics.mat'),'result');
