% PROGRAM DESCRIPTION: Quick-plotting resultant optimal states and controls
% from 'optscript' for comparison
% by Raviraj Nataraj, 20150426
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

close all; clear all;

load result006_BE400.mat;
res400 = Result;
load result006_BE100.mat;
res100 = Result;
load result006_BE50_3.mat;
res50 = Result;

figure(1);
sel_jt = 4; % 4-9
u400 = res400.u(sel_jt,:)';
u100 = res100.u(sel_jt,:)';
u50 = res50.u(sel_jt,:)';
subplot(4,1,1); plot(u400); xlabel('#nodes'); title(['control#', num2str(sel_jt)]);
subplot(4,1,2); plot(u100); xlabel('#nodes');
subplot(4,1,3); plot(u50); xlabel('#nodes');
t400 = (1:length(u400))*(100/length(u400));
t100 = (1:length(u100))*(100/length(u100));
t50 = (1:length(u50))*(100/length(u50));
subplot(4,1,4); plot(t400, u400, t100, u100, t50, u50); xlabel('%gait');

figure(2);
sel_st = 2; % 1-18;
x400 = res400.x(sel_st,:)';
x100 = res100.x(sel_st,:)';
x50 = res50.x(sel_st,:)';
subplot(4,1,1); plot(x400); xlabel('#nodes'); title(['state#', num2str(sel_st)]);
subplot(4,1,2); plot(x100); xlabel('#nodes');
subplot(4,1,3); plot(x50); xlabel('#nodes');
subplot(4,1,4);plot(t400, x400, t100, x100, t50, x50); xlabel('%gait');