close all; clear all;
QR_ratio = 10;
load(['BE50_3\Results_QR',num2str(QR_ratio),'.mat']);
resid = squeeze(Result_optfb.xric(1,:,:))';

subplot(2,2,1); 
plot(resid); %title(num2str(mean(mean(mean(res.xric))))); 
subplot(2,1,2); plot(res.resid); title(num2str(mean(res.resid)));