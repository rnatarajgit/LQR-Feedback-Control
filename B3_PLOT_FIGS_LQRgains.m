% PROGRAM DESCRIPTION: Plotting gain-results of LQR feedback control as a
% function of the gait-cycle
% by Raviraj Nataraj, 20150702
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

function B3_PLOT_FIGS_LQRgains

close all; tic;

% Loading optimal trajectory data
load('gait2dem optim trajec\result006_BE50.mat');
sis = [1:3 7:9 4:6 10:12 16:18 13:15]; % symmetry index parameters for states
x = Result.x; N = size(x,2);
xf(:,(N+1):2*N) = x(sis,1:N); x = xf;

% Label-names for states, joints
st_name = { 'HIP-Xpos','HIP-Ypos','TORS-TILT',...
    'RHIP-ang','RKNE-ang','RANK-ang',...
    'LHIP-ang','LKNE-ang','LANK-ang',...
    'HIP-Xpos-deriv','HIP-Ypos-deriv','TORS-TILT-deriv',...
    'RHIP-ang-deriv','RKNE-ang-deriv','RANK-ang-deriv',...
    'LHIP-ang-deriv','LKNE-ang-deriv','LANK-ang-deriv'};
jt_name = {'RHIP','RKNE','RANK','LHIP','LKNE','LANK'};

% Plotting single gain of select feedback to select joint torque
QR_sel = 1e0;
for QR = QR_sel
    load(['optim feedback gains\BE50\Results_QR',num2str(QR),'.mat']);
end
z = 0;
for jt_sel = 1:3 % 1-6
    figure(jt_sel);
    p = 0; sb_ind = [((1:9)*2-1) (1:9)*2];
    for st_sel = 1:18; % 1-18
        p = p + 1;
        k = squeeze(Result_optfb.k(jt_sel,st_sel,:))'; k = [k(end) k];
        sprintf('%2d %2d %3.2e', jt_sel, st_sel, max(k) - min(k))
        z = z + 1;
        range_k(z) = max(k) - min(k); %save temp range_k;
        sx = subplot(9,2,sb_ind(p));
        x = (1:length(k)) - 1;
        px1 = plot(x, k, 'b'); hold on;
        px2 = plot(x, x*0, 'k:');
        set(sx, 'fontweight', 'bold', 'fontsize', 12);
        set(px1, 'linewidth', 2);
        set(px2, 'linewidth', 2);
    end
end

% Plotting stick-figures across gait-cycle
figure(jt_sel+1); sx = subplot(1,1,1);
% loading optimal trajectory data
load('gait2dem optim trajec\result006_BE50.mat');
sis = [1:3 7:9 4:6 10:12 16:18 13:15]; % symmetry index parameters for states
x = Result.x; N = size(x,2);
xf = x;
xf(:,(N+1):2*N) = x(sis,1:N);
displacement = 0.7550;
xf(1,(N+1):2*N) = x(1,1:N) + displacement; % Adding global x progress from first half gait-cycle
x = xf(1:9,:);
% plot hip-position
%x(1,:) = 1*x(1,:);
px = plot(x(1,:),x(2,:), 'r'); hold on; axis equal;
set(px, 'linewidth', 3);
% plot stick-figure segments
L_torso = 0.6; % 'm'
L_thigh = 0.4410;
L_shank = 0.4428;
L_foot = 0.1;
for t = [35]%[15 40 65 90]% 25 50 75];
    hip = x(1:2,t)';
    % plot torso
    theta = x(3,t) + pi/2;
    head = hip + L_torso*[cos(theta) sin(theta)];
    px = plot([hip(1) head(1)], [hip(2), head(2)], 'k'); hold on; axis equal;
    lw = 3; set(px, 'linewidth', 3);
    % plot left thigh
    theta = -pi/2 + x(3,t) + x(7,t);
    lknee = hip + L_thigh*[cos(theta) sin(theta)];
    px = plot([hip(1) lknee(1)], [hip(2), lknee(2)], 'g-');
    lw = 3; set(px, 'linewidth', 3);
    % plot right thigh
    theta = -pi/2 + x(3,t) + x(4,t);
    rknee = hip + L_thigh*[cos(theta) sin(theta)];
    px = plot([hip(1) rknee(1)], [hip(2), rknee(2)], 'b-');
    lw = 3; set(px, 'linewidth', 3);
    % legend
    lx = legend({'hip-position trace','torso','left leg','right leg'});
    % plot left shank
    theta = -pi/2 + x(3,t) + x(7,t) + x(8,t);
    lank = lknee + L_shank*[cos(theta) sin(theta)];
    px = plot([lknee(1) lank(1)], [lknee(2), lank(2)], 'g-');
    lw = 3; set(px, 'linewidth', lw); set(px, 'linewidth', lw);
    % plot right shank
    theta = -pi/2 + x(3,t) + x(4,t) + x(5,t);
    rank = rknee + L_shank*[cos(theta) sin(theta)];
    px = plot([rknee(1) rank(1)], [rknee(2), rank(2)], 'b-');
    lw = 3; set(px, 'linewidth', lw); set(px, 'linewidth', lw);
    % plot right, left stick-feet
    clear v1 v2;
    ft_down = 0.0502; ft_back = 0.03; ft_front = 0.10;
    v1o = [-ft_back; -ft_down]; v2o = [ft_front; -ft_down];
    p = rank';
    theta = x(3,t) + x(4,t) + x(5,t) + x(6,t);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    v1 = (R*(v1o/norm(v1o)))*norm(v1o) + p;
    v2 = (R*(v2o/norm(v2o)))*norm(v2o) + p;
    lw = 3; px = plot([p(1) v1(1) v2(1) p(1)], [p(2) v1(2) v2(2) p(2)], 'b-'); set(px, 'linewidth', lw);
    p = lank';
    theta = x(3,t) + x(7,t) + x(8,t) + x(9,t);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    v1 = (R*(v1o/norm(v1o)))*norm(v1o) + p;
    v2 = (R*(v2o/norm(v2o)))*norm(v2o) + p;
    lw = 3; px = plot([p(1) v1(1) v2(1) p(1)], [p(2) v1(2) v2(2) p(2)], 'g-'); set(px, 'linewidth', lw);
end
% plot floor
lw = 2; px = plot(x(1,:), zeros(size(x(1,:))), 'r'); set(px, 'linewidth', lw);
% plot phase-lines for begining, end, single-, double-support times of gait cycle for stick-figure
gc_sel = [0 10 50 60 100];
gc_x = gc_sel/100*max(xf(1,:));
max_y = L_torso + max(xf(2,:));
min_y = 0;
max_x = max(xf(1,:)) + L_thigh + L_shank;
min_x = min(xf(1,:)) - L_thigh - L_shank;
axis([min_x max_x min_y max_y]);
for gc = gc_x
    yp = min_y:0.01:max_y;
    xp = gc*ones(size(yp));
    px = plot(xp,yp,'r:');
    set(px, 'linewidth', lw);
end
% plot phase-fills for begining, end, single-, double-support times of gait cycle for stick-figure
% double-support#1
col_sel = [1 0.5 0.5];
x = gc_x(1):(1/100*max(gc_x)):gc_x(2);
a = zeros(size(x)); b = max_y*ones(size(x));
jbfill(x,a,b,col_sel);
% double-support#2
x = gc_x(3):(1/100*max(gc_x)):gc_x(4);
a = zeros(size(x)); b = max_y*ones(size(x));
jbfill(x,a,b,col_sel);
% single-support#1 (right)
col_sel = [1 0 0.25];
x = gc_x(2):(1/100*max(gc_x)):gc_x(3);
a = zeros(size(x)); b = max_y*ones(size(x));
jbfill(x,a,b,col_sel);
% single-support#2 (left)
col_sel = [1 0.25 0];
x = gc_x(4):(1/100*max(gc_x)):gc_x(5);
a = zeros(size(x)); b = max_y*ones(size(x));
jbfill(x,a,b,col_sel);
% plot phase-lines, phase-fills for begining, end, single-, double-support times back onto gain profiles
for jt_sel = 1:3 % 1-6
    gc_x = gc_sel;
    figure(jt_sel);
    p = 0; sb_ind = [((1:9)*2-1) (1:9)*2];
    for st_sel = 1:18; % 1-18
        p = p + 1;
        k = squeeze(Result_optfb.k(jt_sel,st_sel,:))'; k = [k(end) k];
        sx = subplot(9,2,sb_ind(p));
        for gc = gc_sel
            yp = (1.2*min(k)):0.01:(1.2*max(k));
            xp = gc*ones(size(yp));
            px = plot(xp,yp,'r:'); axis tight;
            set(px, 'linewidth', 2);
        end
        min_y = min([0 min(yp)]);
        max_y = max([0 max(yp)]);
        % double-support#1
        col_sel = [1 0.5 0.5];
        x = gc_x(1):(1/100*max(gc_x)):gc_x(2);
        a = min_y*ones(size(x)); b = max_y*ones(size(x));
        jbfill(x,a,b,col_sel);
        % double-support#2
        x = gc_x(3):(1/100*max(gc_x)):gc_x(4);
        a = min_y*ones(size(x)); b = max_y*ones(size(x));
        jbfill(x,a,b,col_sel);
        % single-support#1 (right)
        col_sel = [1 0 0.25];
        x = gc_x(2):(1/100*max(gc_x)):gc_x(3);
        a = min_y*ones(size(x)); b = max_y*ones(size(x));
        jbfill(x,a,b,col_sel);
        % single-support#2 (left)
        col_sel = [1 0.25 0];
        x = gc_x(4):(1/100*max(gc_x)):gc_x(5);
        a = min_y*ones(size(x)); b = max_y*ones(size(x));
        jbfill(x,a,b,col_sel);
    end
end
toc

end