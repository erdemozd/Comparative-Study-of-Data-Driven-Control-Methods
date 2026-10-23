clear;
clc;
close all;

p = three_mass_params();
s = benchmark_settings(p);

projectRoot = fileparts(fileparts(mfilename('fullpath')));

deepcFile = fullfile(projectRoot,'data','deepc_training_data.mat');

if exist(deepcFile,'file')
    load(deepcFile,'deepcData');
    uData = deepcData.u;
else
    Tdata = 900;
    uData = generate_excitation(Tdata,8.0);
end
noisyData = uData + 0*randn(size(uData));
Tdata = size(uData,2);

xTrue = zeros(6,Tdata);
xNom = zeros(6,Tdata);

[A,B,C,D] = three_mass_nominal_linear(p);
sysd = c2d(ss(A,B,C,D),p.Ts,'zoh');

Ad = sysd.A;
Bd = sysd.B;

for k = 1:Tdata-1
    xTrue(:,k+1) = rk4_step(xTrue(:,k),noisyData(k),p.Ts,p);
    xNom(:,k+1) = Ad*xNom(:,k) + Bd*noisyData(k);
end
%% output noise
signalRMS = sqrt(mean(xTrue.^2,2));
noiseLevel = 0.02;
sigmaX = noiseLevel * signalRMS;
noiseX = sigmaX .* randn(size(xTrue));

xTrueNoisy = noiseX + xTrue;
xResidual = xTrueNoisy - xNom;

sdmpcData.u = noisyData;
sdmpcData.xTrue = xTrueNoisy;
sdmpcData.xNom = xNom;
sdmpcData.xResidual = xResidual;
%%
sdmpcData.Tini = s.Tini;
sdmpcData.N = s.N;
sdmpcData.Ts = p.Ts;

dataFile = fullfile(projectRoot,'data','sdmpc_residual_data.mat');

save(dataFile,'sdmpcData');

fprintf('\nSD-MPC residual data saved:\n%s\n\n',dataFile);

t = (0:Tdata-1)'*p.Ts;

figure('Color','w','Name','Three-Mass DeePC Data');
subplot(4,1,1);
plot(t,uData');
grid on;
ylabel('u [N]');
title('Not Used Clean Training Input');


subplot(4,1,2);
plot(t,noisyData');
grid on;
ylabel('u [N]');
title('Used Noisy Training Input');

subplot(4,1,3);
plot(t,xTrue([1 3 5],:)');
grid on;
ylabel('Position [m]');
legend('q1','q2','q3');

subplot(4,1,4);
plot(t,xTrueNoisy([1 3 5],:)');
grid on;
ylabel('Position [m]');
legend('q1','q2','q3');

figure;
subplot(2,1,1)
plot(t,xResidual([1 3 5],:)');
grid on;
ylabel('Residual Position [m]');
legend('rq1','rq2','rq3');


nomResidual = xTrue - xNom;
subplot(2,1,2)
plot(t,nomResidual([1 3 5],:))
ylabel('Nominal Residual Position [m]');
legend('nrq1','nrq2','nrq3');