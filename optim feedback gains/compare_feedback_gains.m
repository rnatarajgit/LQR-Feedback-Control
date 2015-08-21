% PROGRAM DESCRIPTION: Quick-plotting resultant feedback gains from dpre.m
% for comparison across optimal trajectories
% by Raviraj Nataraj, 20150426
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

close all; clear all;

QR_ratio = 1e2;
load(['BE400\Results_QR',num2str(QR_ratio),'.mat']);
res = Result_optfb;
k_BE400 = res.k;
load(['BE50_3\Results_QR',num2str(QR_ratio),'.mat']);
res = Result_optfb;
k_BE50 = res.k;

figure;
for f = 1:6
    subplot(6,1,f);    
    jt = f; % 1-6
    st = 5; % 1-18
    k1 = squeeze(k_BE400(jt,st,:)); inc = 100/(length(k1)-1); t1 = 0:inc:100;
    k2 = squeeze(k_BE50(jt,st,:)); inc = 100/(length(k2)-1); t2 = 0:inc:100;
    px = plot(t1,k1,t2,k2); set(px, 'linewidth', 3); lx = legend({'BE400', 'BE50'});
    tx = title(['state# ',num2str(st)]);
    bx = ylabel(['joint# ',num2str(jt)]);
end