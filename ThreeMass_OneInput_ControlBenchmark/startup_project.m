clear;
clc;

projectRoot = fileparts(mfilename('fullpath'));

addpath(fullfile(projectRoot,'common'));
addpath(fullfile(projectRoot,'MPC'));
addpath(fullfile(projectRoot,'DeePC'));
addpath(fullfile(projectRoot,'SD-MPC'));
addpath(fullfile(projectRoot,'HDeePC'));
addpath(fullfile(projectRoot,'PZ-DeePC'));
addpath(fullfile(projectRoot,'Comparison'));

fprintf('\nThree-mass benchmark paths added.\n');
fprintf('Project root: %s\n\n',projectRoot);
