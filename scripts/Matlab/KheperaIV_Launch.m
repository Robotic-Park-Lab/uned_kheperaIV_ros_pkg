%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Khepera IV Set up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables;
% clc;
close all;

disp('********************')
disp('*    Khepera IV    *')
disp('********************')
disp('')
% PIDc  -> PID continuo
% PIDd  -> PID discreto
% PIDeb -> PID Basado en Eventos
controller = "PIDeb";
%% Model
disp('Modelling ...')
tic
KheperaIV_Model
tm = toc;
fprintf('\tModelling time: %f\n', tm)