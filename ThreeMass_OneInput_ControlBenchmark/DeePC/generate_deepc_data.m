clear;
clc;
close all;

p = three_mass_params();
s = benchmark_settings(p);

Tdata = 600;
amplitude = 8.0;

rng(1);

uData = generate_excitation(Tdata,amplitude);

%% Input noise
inputNoiseLevel = 0.00;
signalRMS_U = sqrt(mean(uData.^2));
sigmaU = inputNoiseLevel*signalRMS_U;
inputNoise = sigmaU*randn(size(uData));
uMeasured = uData + inputNoise;

t = (0:Tdata-1)'*p.Ts;

%% Clean nonlinear plant trajectory
yData = zeros(6,Tdata);

for k = 1:Tdata-1
    yData(:,k+1) = rk4_step(yData(:,k),uMeasured(k),p.Ts,p);
end

%% Output measurement noise
signalRMS = sqrt(mean(yData.^2,2));
outputNoiseLevel = 0.02;

sigmaY = outputNoiseLevel*signalRMS;
outputNoise = sigmaY.*randn(size(yData));
yNoisy = yData + outputNoise;

figure('Color','w','Name','Noisy Output Measurements');

subplot(6,1,1)
plot(t,yNoisy(1,:));
grid on;
ylabel('q_1 [m]');

subplot(6,1,2)
plot(t,yNoisy(2,:));
grid on;
ylabel('v_1 [m/s]');

subplot(6,1,3)
plot(t,yNoisy(3,:));
grid on;
ylabel('q_2 [m]');

subplot(6,1,4)
plot(t,yNoisy(4,:));
grid on;
ylabel('v_2 [m/s]');

subplot(6,1,5)
plot(t,yNoisy(5,:));
grid on;
ylabel('q_3 [m]');

subplot(6,1,6)
plot(t,yNoisy(6,:));
grid on;
ylabel('v_3 [m/s]');
xlabel('Time [s]');

%% PE check
L = s.Tini+s.N;
peOrder = L+6;

Hpe = block_hankel(uMeasured,peOrder);
rankPE = rank(Hpe);

%% Nonlinear spring exposure
dq23 = yData(5,:)-yData(3,:);
ratio = 100*(p.k3/p.k23)*dq23.^2;

%% Store data
deepcData.u = uMeasured;
deepcData.y = yNoisy;

deepcData.Tini = s.Tini;
deepcData.N = s.N;
deepcData.Ts = p.Ts;

deepcData.peOrder = peOrder;
deepcData.rankPE = rankPE;
deepcData.maxNonlinearRatio = max(ratio);

deepcData.uClean = uData;
deepcData.yClean = yData;

deepcData.inputNoise = inputNoise;
deepcData.outputNoise = outputNoise;

deepcData.inputNoiseLevel = inputNoiseLevel;
deepcData.outputNoiseLevel = outputNoiseLevel;

deepcData.signalRMS_U = signalRMS_U;
deepcData.signalRMS_Y = signalRMS;

deepcData.sigmaU = sigmaU;
deepcData.sigmaY = sigmaY;

projectRoot = fileparts(fileparts(mfilename('fullpath')));
dataFile = fullfile(projectRoot,'data','deepc_training_data.mat');

save(dataFile,'deepcData');

fprintf('\nDEEPC DATA GENERATED\n');
fprintf('============================================================\n');
fprintf('Tdata: %d\n',Tdata);
fprintf('PE rank: %d / %d\n',rankPE,peOrder);

fprintf('\nINPUT NOISE\n');
fprintf('Input noise level: %.2f %% of input RMS\n',100*inputNoiseLevel);
fprintf('Input RMS: %.6f N\n',signalRMS_U);
fprintf('Input noise sigma: %.6f N\n',sigmaU);

fprintf('\nOUTPUT NOISE\n');
fprintf('Output noise level: %.2f %% of channel RMS\n',100*outputNoiseLevel);
fprintf('q1 RMS / sigma: %.6f / %.6f m\n',signalRMS(1),sigmaY(1));
fprintf('v1 RMS / sigma: %.6f / %.6f m/s\n',signalRMS(2),sigmaY(2));
fprintf('q2 RMS / sigma: %.6f / %.6f m\n',signalRMS(3),sigmaY(3));
fprintf('v2 RMS / sigma: %.6f / %.6f m/s\n',signalRMS(4),sigmaY(4));
fprintf('q3 RMS / sigma: %.6f / %.6f m\n',signalRMS(5),sigmaY(5));
fprintf('v3 RMS / sigma: %.6f / %.6f m/s\n',signalRMS(6),sigmaY(6));

fprintf('\nTRUE PLANT EXCITATION\n');
fprintf('Max |q1|: %.4f m\n',max(abs(yData(1,:))));
fprintf('Max |q2|: %.4f m\n',max(abs(yData(3,:))));
fprintf('Max |q3|: %.4f m\n',max(abs(yData(5,:))));
fprintf('Max |q3-q2|: %.4f m\n',max(abs(dq23)));
fprintf('Max cubic/linear ratio: %.2f %%\n',max(ratio));
fprintf('Saved: %s\n\n',dataFile);

%% Training data plots
figure('Color','w','Name','Three-Mass DeePC Data');

subplot(5,1,1);
plot(t,uData');
grid on;
ylabel('u [N]');
title('Clean Training Input');

subplot(5,1,2);
plot(t,uMeasured');
grid on;
ylabel('u [N]');
title('Measured / Used Training Input');

subplot(5,1,3);
plot(t,yData([1 3 5],:)');
grid on;
ylabel('Position [m]');
title('Clean Output Signals');
legend('q1','q2','q3');

subplot(5,1,4);
plot(t,yNoisy([1 3 5],:)');
grid on;
ylabel('Position [m]');
title('Noisy Output Signals');
legend('q1','q2','q3');

subplot(5,1,5);
plot(t,ratio');
grid on;
xlabel('Time [s]');
ylabel('Nonlinear ratio [%]');
title('True Plant Nonlinear Spring Exposure');

%% Noise plots
figure('Color','w','Name','DeePC Output Measurement Noise');

labels = {'q_1','v_1','q_2','v_2','q_3','v_3'};

for i = 1:6
    subplot(6,1,i);
    plot(t,outputNoise(i,:),'LineWidth',0.8);
    grid on;

    if mod(i,2)==1
        ylabel('Noise [m]');
    else
        ylabel('Noise [m/s]');
    end

    title(sprintf('%s measurement noise, sigma = %.4g',labels{i},sigmaY(i)));
end

xlabel('Time [s]');