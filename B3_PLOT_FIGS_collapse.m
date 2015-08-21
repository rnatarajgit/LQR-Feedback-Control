% PROGRAM DESCRIPTION: Plotting results of LQR closed-loop torque response 
% to prescribed hip-collapse at particular point in gait-cycle 
% by Raviraj Nataraj, 20150529
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

function B3_PLOT_FIGS_collapse

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
QR_ratio = 1; % Q/R ratio for LQR gain results being utilized
gait_in = 60; % input point in gait-cycle (1-100%)
hcd_in = [0:0.006:0.03]; % input hip-collapse displacement
hcv_in = hcd_in/0.03;% sqrt(2*9.81*hcd_in); % assuming constant acceleration (9.81m/s^2) and zero initial velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Adding path with collapse analysis files
addpath('collapse analysis');

% Computing collapse torques, related data given inputs
[err, toq, pos, ang1] = compute_collapse_torques(QR_ratio, gait_in, hcd_in, hcv_in);
close(figure(1)); close(figure(2)); close(figure(3)); 

% Computing mean joint torque values for plotting
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

% Plotting collapse stick-figures and stance leg joint torques
figure;
sx1 = subplot(1,1,1);
% plot stick-torso
torso_ht = 0.6; % 'm'
theta = ang1.torso; % radians
orig = pos{1}.lt_leg1(:,1);
v = [cos(theta) -sin(theta); sin(theta) cos(theta)]*[0 1]'*torso_ht + orig;
px = plot([orig(1) v(1)],[orig(2) v(2)], 'k'); set(px, 'linewidth', 4);
hold on; axis equal;
% plot stick-legs
n = length(hcd_in);
for pt = n:-1:1 
    k = (pt-1)*n + 1;
    rt_leg1 = pos{k}.rt_leg1; lt_leg1 = pos{k}.lt_leg1; rt_leg2 = pos{k}.rt_leg2; lt_leg2 = pos{k}.lt_leg2;
    px2 = plot(lt_leg1(1,:), lt_leg1(2,:),'go-');
    px1 = plot(rt_leg1(1,:), rt_leg1(2,:),'bo-'); 
    axis([0.4 1.3 0 2.0]);    
    set(px1, 'linewidth', 4);
    set(px2, 'linewidth', 4);
    px1 = plot(rt_leg2(1,:), rt_leg2(2,:),'bo:');
    px2 = plot(lt_leg2(1,:), lt_leg2(2,:),'go:');
    set(px1, 'linewidth', 3);
    set(px2, 'linewidth', 3);
end
lx = legend({'torso', 'stance leg',' swing leg'});
% plot left stick-foot
L = pos{1}.lt_leg1; 
ft_back = 0.03; ft_front = 0.10;
px = plot([L(1,3) L(1,3)-ft_back L(1,3)+ft_front L(1,3)], [L(2,3) 0 0 L(2,3)], 'g-'); set(px, 'linewidth', 4); 
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
px = plot([R(1,3) v1(1) v2(1) R(1,3)], [R(2,3) v1(2) v2(2) R(2,3)], 'b-'); set(px, 'linewidth', 4); 
ax = xlabel('horizontal position (m)');
bx = ylabel('vertical position (m)');
tx = title('Single-Leg Stance Collapse');
set(sx1, 'fontsize', 12, 'fontweight', 'bold');
set(ax, 'fontsize', 15, 'fontweight', 'bold');
set(bx, 'fontsize', 15, 'fontweight', 'bold');
set(tx, 'fontsize', 18, 'fontweight', 'bold');

% plot leg torques
figure;
jt_dof = {'swing hip-flex', 'swing knee-ext', 'swing ank-df', 'stance HIP-EXT', 'stance KNEE-EXT', 'stance ANKLE-PF' };
k = 0;
for jt = 4:6
    k = k + 1;
    x = 1e2*hcd_in;
    y = abs(mean_toq_lqr(jt,:));
    e = std_toq_lqr(jt,:);
    sx2 = subplot(1,3,jt-3);    
    px1 = errorbar(x,y,e,'ro-'); hold on;       
    set(px1, 'linewidth', 1.5); axis([min(x)-1e-1 max(x)+1e-1 0 250]);
    set(sx2, 'fontsize', 12, 'fontweight', 'bold');
    tx = title(jt_dof{jt}); set(tx, 'fontsize', 18, 'fontweight', 'bold'); 
    switch(k)
        case{1}            
            bx = ylabel('LQR closed-loop torque (N-m)');
            set(bx, 'fontsize', 15, 'fontweight', 'bold');            
        case{2}
            ax = xlabel('hip collapse displacement (cm)');             
            set(ax, 'fontsize', 15, 'fontweight', 'bold');            
        case{3}
            lx = legend('std.dev. across varying collapse velocities (0-100 cm/sec)');
            set(lx, 'fontsize', 12, 'fontweight', 'bold');
    end
end

toc

end