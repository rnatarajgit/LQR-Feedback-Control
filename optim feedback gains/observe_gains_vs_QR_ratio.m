% PROGRAM DESCRIPTION: Observing optimal feedback gains across Q/R's
% by Raviraj Nataraj, 20150604
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

clear all; close all;

% Plotting optimal feedback gains for select single state across select Q/R values
st_sel = 3; % 1-18
QR_sel = [1e-1 2e-1 5e-1 1e0 2e0 5e0 1e1 2e1 5e1 1e2];%[1e-1 2e-1 5e-1 1e0 2e0 5e0 1e1 2e1 5e1 1e2];
for i = 1:length(QR_sel)
    QR_leg{i} = num2str(QR_sel(i));
end
figure(1);
for QR = QR_sel
    load(['BE50\Results_QR',num2str(QR),'.mat']);
    K = Result_optfb.k;
    for jt = 1:6
        subplot(6,1,jt);
        st = st_sel;
        k = squeeze(K(jt,st,:));
        t = 1:length(k);
        px = plot(t,k); set(px, 'linewidth', 3); hold on;
        tx = title(['state# ',num2str(st)]);
        bx = ylabel(['joint# ',num2str(jt)]);
    end
end
legend(QR_leg);

% Plotting scaling between minimum and maximum selected Q/R
figure(2);
load(['BE50\Results_QR',num2str(min(QR_sel)),'.mat']);
K1 = Result_optfb.k;
load(['BE50\Results_QR',num2str(max(QR_sel)),'.mat']);
K2 = Result_optfb.k;
Sf_base = max(QR_sel)/min(QR_sel);
for jt = 1:6
    figure(2); subplot(6,1,jt);
    st = st_sel;
    k1 = squeeze(K1(jt,st,:));
    k2 = squeeze(K2(jt,st,:));
    Sf = k2./k1;
    t = 1:length(k1);
    px = plot(t,k1,t,k2); set(px, 'linewidth', 3); hold on;
    tx = title(['state# ',num2str(st)]);
    bx = ylabel(['joint# ',num2str(jt)]);
    figure(3); subplot(6,1,jt);
    px = plot(t,Sf,t,Sf_base*ones(size(t)));
end