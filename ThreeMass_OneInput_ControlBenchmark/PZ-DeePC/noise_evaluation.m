clear; clc; close all;

p = three_mass_params();

projectRoot = fileparts(fileparts(mfilename('fullpath')));

if isempty(projectRoot)
    projectRoot = pwd;
end

dataFile = fullfile(projectRoot,'data','deepc_training_data.mat');

if ~exist(dataFile,'file')
    error('Run DeePC/generate_deepc_data.m first.');
end

load(dataFile,'deepcData');

if ~isfield(deepcData,'yClean')
    error('deepcData.yClean is required for this test.');
end

nKnown = 4;

prior = pz_known_poles(p,nKnown);

ny = 6;
Ts = p.Ts;

yClean = deepcData.yClean;
yNoisy = deepcData.y;

T = size(yClean,2);

if size(yNoisy,2) ~= T
    error('Clean and noisy datasets must have the same length.');
end

%% Raw measurement noise

noiseRaw = yNoisy-yClean;

%% Filter clean and noisy signals

yCleanPrimeVec = pz_filter_past_output( ...
    yClean(:), ...
    prior.polyCoeff, ...
    ny);

yNoisyPrimeVec = pz_filter_past_output( ...
    yNoisy(:), ...
    prior.polyCoeff, ...
    ny);

sp = prior.sp;
Tprime = T-sp;

yCleanPrime = reshape(yCleanPrimeVec,ny,Tprime);
yNoisyPrime = reshape(yNoisyPrimeVec,ny,Tprime);

noiseFiltered = yNoisyPrime-yCleanPrime;

%% Statistics

stdRaw = std(noiseRaw,0,2);
stdFiltered = std(noiseFiltered,0,2);

rmsRaw = sqrt(mean(noiseRaw.^2,2));
rmsFiltered = sqrt(mean(noiseFiltered.^2,2));

stdGain = stdFiltered./stdRaw;
rmsGain = rmsFiltered./rmsRaw;

energyRaw = sum(noiseRaw(:).^2);
energyFiltered = sum(noiseFiltered(:).^2);

energyGain = energyFiltered/energyRaw;

labels = {'q_1','v_1','q_2','v_2','q_3','v_3'};

fprintf('\nPZ-DeePC NOISE FILTER STUDY\n');
fprintf('===============================================================\n');
fprintf('Known poles             : %d\n',nKnown);
fprintf('Filter order            : %d\n',sp);
fprintf('Raw noise samples       : %d\n',T);
fprintf('Filtered noise samples  : %d\n',Tprime);
fprintf('Overall energy gain     : %.4f\n\n',energyGain);

fprintf('%-6s %12s %12s %12s %12s\n', ...
    'State','STD raw','STD filt','STD gain','RMS gain');

fprintf('---------------------------------------------------------------\n');

for i = 1:ny
    fprintf('%-6s %12.4e %12.4e %12.4f %12.4f\n', ...
        labels{i}, ...
        stdRaw(i), ...
        stdFiltered(i), ...
        stdGain(i), ...
        rmsGain(i));
end

fprintf('===============================================================\n');

%% Time vectors

tRaw = (0:T-1)*Ts;
tFiltered = (sp:T-1)*Ts;

%% Raw and filtered noise

figure('Color','w','Name','PZ-DeePC Noise Filter Study - Time Signals');

tl = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

for i = 1:ny

    nexttile;

    plot(tRaw,noiseRaw(i,:),'LineWidth',0.8);
    hold on;
    plot(tFiltered,noiseFiltered(i,:),'LineWidth',0.8);

    grid on;

    xlabel('Time [s]');

    if mod(i,2)==1
        ylabel('Noise [m]');
    else
        ylabel('Noise [m/s]');
    end

    title(labels{i});

    legend('Raw white noise','Filtered noise','Location','best');

end

title(tl,sprintf('%d-Pole PZ-DeePC Noise Transformation',nKnown));

%% Noise standard deviation comparison

figure('Color','w','Name','PZ-DeePC Noise Filter Study - Noise Magnitude');

tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;

bar([stdRaw stdFiltered]);

grid on;

set(gca,'XTick',1:ny);
set(gca,'XTickLabel',labels);

ylabel('Standard deviation');
title('Raw and Filtered Noise Standard Deviation');

legend('Raw','Filtered','Location','best');

nexttile;

bar(stdGain);

grid on;

set(gca,'XTick',1:ny);
set(gca,'XTickLabel',labels);

ylabel('\sigma_{filtered} / \sigma_{raw}');
title('Noise Standard-Deviation Amplification');

yline(1,'k--');

%% Power spectral density

Nfft = 2^nextpow2(max(T,Tprime));

f = (0:Nfft/2)*(1/(Ts*Nfft));

figure('Color','w','Name','PZ-DeePC Noise Filter Study - PSD');

tl = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

for i = 1:ny

    rawFFT = fft(noiseRaw(i,:),Nfft);
    filtFFT = fft(noiseFiltered(i,:),Nfft);

    rawPSD = abs(rawFFT).^2/T;
    filtPSD = abs(filtFFT).^2/Tprime;

    rawPSD = rawPSD(1:Nfft/2+1);
    filtPSD = filtPSD(1:Nfft/2+1);

    nexttile;

    semilogy(f,max(rawPSD,1e-18),'LineWidth',1.0);
    hold on;
    semilogy(f,max(filtPSD,1e-18),'LineWidth',1.0);

    grid on;

    xlabel('Frequency [Hz]');
    ylabel('Power');

    title(labels{i});

    legend('Raw noise','Filtered noise','Location','best');

end

title(tl,'Noise Power Spectral Density');

%% Normalized autocorrelation

maxLag = 30;

figure('Color','w','Name','PZ-DeePC Noise Filter Study - Autocorrelation');

tl = tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

for i = 1:ny

    raw = noiseRaw(i,:);
    filt = noiseFiltered(i,:);

    raw = raw-mean(raw);
    filt = filt-mean(filt);

    acRaw = zeros(maxLag+1,1);
    acFilt = zeros(maxLag+1,1);

    for lag = 0:maxLag
        acRaw(lag+1) = mean(raw(1:end-lag).*raw(1+lag:end));
        acFilt(lag+1) = mean(filt(1:end-lag).*filt(1+lag:end));
    end

    acRaw = acRaw/acRaw(1);
    acFilt = acFilt/acFilt(1);

    nexttile;

    stem(0:maxLag,acRaw,'filled');
    hold on;
    plot(0:maxLag,acFilt,'o-','LineWidth',1.0);

    grid on;

    xlabel('Lag [samples]');
    ylabel('Normalized autocorrelation');

    title(labels{i});

    legend('Raw noise','Filtered noise','Location','best');

end

title(tl,'Noise Autocorrelation');

%% Filter frequency response

w = linspace(0,pi,1000);
zUnit = exp(1j*w);
Hfilter = polyval(prior.polyCoeff,zUnit);

figure('Color','w','Name','PZ-DeePC Noise Filter Study - Filter Response');

plot(w/pi,abs(Hfilter),'LineWidth',1.4);

grid on;

xlabel('\omega/\pi');
ylabel('|P(e^{j\omega})|');

title(sprintf('%d-Pole PZ Filter Magnitude',nKnown));