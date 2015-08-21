% PROGRAM DESCRIPTION: Collecting collapse errors, torques across multiple gait cycle positions 
% by Raviraj Nataraj, 20150623
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

close all; clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
gait_in = 60; % input point in gait-cycle (1-100%)
hcd_in = [0:0.006:0.03]; % input hip-collapse displacement
hcv_in = sqrt(2*9.81*hcd_in); % assuming constant acceleration (9.81m/s^2) and zero initial velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[err, toq, pos, ang1] = compute_collapse_torques(1, gait_in, hcd_in, hcv_in);

% Plotting mesh plots
figure(4);
X = err.hcd(1:length(hcd_in));
Y = err.hcv(1:length(hcv_in));
p1 = [1 5 9 3 7 11];
p2 = p1 + 1;
jt_label{1} = 'rt-hip (flexion)';
jt_label{2} = 'rt-kne (extension)';
jt_label{3} = 'rt-ank (dorsi-flexion)';
jt_label{4} = 'lt-hip (flexion)';
jt_label{5} = 'lt-kne (extension)';
jt_label{6} = 'lt-ank (dorsi-flexion)';
for jt = 1:6;
    k = 0;
    for i = 1:length(hcd_in)
        for j = 1:length(hcv_in)
            k = k + 1;
            Z1(i,j) = toq.lqr(k,jt).^1;
            Z2(i,j) = toq.pd(k,jt).^1;
            X(i,j) = err.hcd(k);
            Y(i,j) = err.hcv(k);
        end
    end
    X = X(1:length(hcd_in),1);
    Y = Y(1,1:length(hcd_in));
    minZ = ones(1,6)*min(min([Z1; Z2]));
    maxZ = ones(1,6)*max(max([Z1; Z2]));
    subplot(3,4,p1(jt)); surfc(X,Y,Z1); title(['LQR ',jt_label{jt}]); xlabel('hcd (m)'); ylabel('hcv (m/sec)'); zlabel('torque (N-m)'); axis([min(X) max(X) min(Y) max(Y) minZ(jt) maxZ(jt)]);
    subplot(3,4,p2(jt)); surfc(X,Y,Z2); title(['PD ',jt_label{jt}]);  xlabel('hcd (m)'); ylabel('hcv (m/sec)'); zlabel('torque (N-m)'); axis([min(X) max(X) min(Y) max(Y) minZ(jt) maxZ(jt)]);
end

% Plotting simple joint torque plot
figure(5);
for jt = 1:6
    n = length(hcd_in);
    for k = 1:n
        p1 = (k-1)*n + 1;
        p2 = p1 + n - 1;
        toq_sel = toq.lqr(p1:p2,jt);
        mean_toq_lqr(jt,k) = mean(toq_sel);
        std_toq_lqr(jt,k) = std(toq_sel);
        toq_sel = toq.pd(p1:p2,jt);
        mean_toq_pd(jt,k) = mean(toq_sel);
        std_toq_pd(jt,k) = std(toq_sel);
    end
end
pc = {'b:','g:','r:','b-','g-','r-'}; % plot colors
subplot(1,2,1);
for jt = 1:6
    errorbar(hcd_in, mean_toq_lqr(jt,:), std_toq_lqr(jt,:), pc{jt}); hold on; axis([min(hcd_in) max(hcd_in) -600 1000]);
end
legend('rt-hip', 'rt-kne', 'rt-ank', 'lt-hip', 'lt-kne', 'lt-ank');
subplot(1,2,2);
for jt = 1:6
    errorbar(hcd_in, mean_toq_pd(jt,:), std_toq_pd(jt,:), pc{jt}); hold on; axis([min(hcd_in) max(hcd_in) -600 1000]);
end
legend('rt-hip', 'rt-kne', 'rt-ank', 'lt-hip', 'lt-kne', 'lt-ank');

% Plotting collapse stick-figures and stance leg joint torques
figure(6);
subplot(1,4,1);
% plot stick-torso
torso_ht = 0.6; % 'm'
theta = ang1.torso; % radians
orig = pos{1}.lt_leg1(:,1);
v = [cos(theta) -sin(theta); sin(theta) cos(theta)]*[0 1]'*torso_ht + orig;
px = plot([orig(1) v(1)],[orig(2) v(2)], 'k'); set(px, 'linewidth', 6);
hold on; axis equal;
% plot stick-legs
n = length(hcd_in);
for pt = n:-1:1 
    k = (pt-1)*n + 1;
    rt_leg1 = pos{k}.rt_leg1; lt_leg1 = pos{k}.lt_leg1; rt_leg2 = pos{k}.rt_leg2; lt_leg2 = pos{k}.lt_leg2;
    px2 = plot(lt_leg1(1,:), lt_leg1(2,:),'go-');
    px1 = plot(rt_leg1(1,:), rt_leg1(2,:),'bo-'); 
    axis([0.4 1.3 0 2.0]);    
    set(px1, 'linewidth', 6);
    set(px2, 'linewidth', 6);
    px1 = plot(rt_leg2(1,:), rt_leg2(2,:),'bo:');
    px2 = plot(lt_leg2(1,:), lt_leg2(2,:),'go:');
    set(px1, 'linewidth', 1);
    set(px2, 'linewidth', 1);
end
legend({'torso', 'swing leg',' stance leg'});
% plot left stick-foot
L = pos{1}.lt_leg1; 
ft_back = 0.03; ft_front = 0.10;
px = plot([L(1,3) L(1,3)-ft_back L(1,3)+ft_front L(1,3)], [L(2,3) 0 0 L(2,3)], 'g-'); set(px, 'linewidth', 6); 
ft_down = L(2,3);
% plot right stick-foot
R = pos{1}.rt_leg1; 
v1(1) = - ft_back;
v1(2) = - ft_down;
v2(1) = ft_front;
v2(2) = - ft_down;
theta = ang1.rhip + ang1.rkne + ang1.rank; % radians
Rm = [cos(theta) -sin(theta); sin(theta) cos(theta)];
v = v1'/norm(v1);
v1 = (Rm*v)*norm(v1) + R(:,3);
v = v2'/norm(v2);
v2 = (Rm*v)*norm(v2) + R(:,3);
px = plot([R(1,3) v1(1) v2(1) R(1,3)], [R(2,3) v1(2) v2(2) R(2,3)], 'b-'); set(px, 'linewidth', 6); 

% plot leg torques
jt_dof = {'swing hip-flex', 'swing knee-ext', 'swing ank-df', 'stance hip-flex', 'stance knee-ext', 'stance ank-df' };
for jt = 4:6
    x = 1e2*hcd_in;
    y = abs(mean_toq_lqr(jt,:));
    e = std_toq_lqr(jt,:);
    subplot(1,4,jt-2);
    px1 = errorbar(x,y,e,'ro-'); hold on;
    %px2 = plot(x,y,'r');
    set(px1, 'linewidth', 3); axis([min(x)-1e-1 max(x)+1e-1 0 300]);
    title(jt_dof{jt});
    xlabel('hip collapse displacement (cm)');
    ylabel('joint torque (N-m)');
end

toc