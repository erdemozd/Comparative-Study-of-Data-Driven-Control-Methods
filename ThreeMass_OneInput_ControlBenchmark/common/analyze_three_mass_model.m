clear;
clc;
close all;

p = three_mass_params();

[A,B,C,D] = three_mass_nominal_linear(p);

sysc = ss(A,B,C,D);
sysd = c2d(sysc,p.Ts,'zoh');

Ad = sysd.A;
Bd = sysd.B;

fprintf('\n');
fprintf('THREE-MASS NOMINAL / LOCAL MODEL ANALYSIS\n');
fprintf('=========================================\n\n');

fprintf('Parameters\n');
fprintf('----------\n');
fprintf('m1 = %.3f kg, m2 = %.3f kg, m3 = %.3f kg\n',p.m1,p.m2,p.m3);
fprintf('k1 = %.3f, k12 = %.3f, k23 = %.3f N/m\n',p.k1,p.k12,p.k23);
fprintf('k3 = %.3f N/m^3\n',p.k3);
fprintf('c1 = %.3f, c12 = %.3f, c23 = %.3f N s/m\n',p.c1,p.c12,p.c23);
fprintf('Ts = %.3f s\n\n',p.Ts);

pc = eig(A);
pd = eig(Ad);

[~,idxc] = sort(abs(imag(pc)));
pc = pc(idxc);

[~,idxd] = sort(abs(angle(pd)));
pd = pd(idxd);

fprintf('Continuous poles\n');
fprintf('----------------\n');
for i = 1:numel(pc)
    fprintf('%2d: % .8f %+.8fi\n',i,real(pc(i)),imag(pc(i)));
end

fprintf('\nDiscrete poles\n');
fprintf('--------------\n');
for i = 1:numel(pd)
    freq = abs(angle(pd(i)))/(2*pi*p.Ts);
    fprintf('%2d: % .8f %+.8fi   |z|=%.8f   f=%.5f Hz\n', ...
        i,real(pd(i)),imag(pd(i)),abs(pd(i)),freq);
end

Ctrb = ctrb(Ad,Bd);
Obsv = obsv(Ad,eye(6));

fprintf('\nControllability rank: %d / 6\n',rank(Ctrb));
fprintf('Observability rank  : %d / 6\n',rank(Obsv));

%% Tracking-channel zeros: u -> q3

Cq3 = [0 0 0 0 1 0];
Dq3 = 0;

sysQ3c = ss(A,B,Cq3,Dq3);
sysQ3d = c2d(sysQ3c,p.Ts,'zoh');

zc = tzero(sysQ3c);
zd = tzero(sysQ3d);

fprintf('\nContinuous transmission zeros for u -> q3\n');
fprintf('-----------------------------------------\n');
disp(zc);

fprintf('Discrete transmission zeros for u -> q3\n');
fprintf('---------------------------------------\n');
disp(zd);

%% Position-channel zeros for reference

positionRows = [1 3 5];
positionNames = {'q1','q2','q3'};

for j = 1:3
    Cj = zeros(1,6);
    Cj(positionRows(j)) = 1;

    sc = ss(A,B,Cj,0);
    sd = c2d(sc,p.Ts,'zoh');

    fprintf('\nZeros u -> %s (continuous):\n',positionNames{j});
    disp(tzero(sc));

    fprintf('Zeros u -> %s (discrete):\n',positionNames{j});
    disp(tzero(sd));
end

%% Pole plot

figure('Color','w','Name','Three-Mass Discrete Pole Map');

theta = linspace(0,2*pi,500);
plot(cos(theta),sin(theta),'k--','LineWidth',1.0);
hold on;
grid on;
axis equal;

plot(real(pd),imag(pd),'x','MarkerSize',10,'LineWidth',1.8);

xlabel('Re(z)');
ylabel('Im(z)');
title('Nominal Discrete Poles');
xlim([-1.1 1.1]);
ylim([-1.1 1.1]);

%% Nonlinearity ratio
%
% |F_cubic| / |F_linear| = (k3/k23)*dq23^2

dq = linspace(-0.5,0.5,1000);

ratio = 100*(p.k3/p.k23)*dq.^2;

figure('Color','w','Name','Three-Mass Nonlinearity Ratio');
plot(dq,ratio,'LineWidth',1.5);
grid on;
xlabel('q_3-q_2 [m]');
ylabel('|F_{cubic}|/|F_{linear}| [%]');
title('Nonlinear Coupling Strength');
yline(100,'k--');
