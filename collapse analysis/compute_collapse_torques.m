% PROGRAM DESCRIPTION: Inverse analysis to identify fall/collapse
% trajectories for grounded leg from optimal walking pattern
% by Raviraj Nataraj, 20150606
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

% Script function to compute and plot joint angle errors, control torques, and joint positions for prescribed collapses of hip-joint at select optimal trajectory points
function [err, toq, pos, ang1] = compute_collapse_torques(QR_ratio, ti_in, hcd_in, hcv_in) 
% QR_ratio = 1 (need other Results_QR mat-files for other chocies)
% ti_in = 1-100 (%gait-cycle)
% hcd_in = recommended hip-collapse distance range: 0.0 - 0.020 m
% hcv_in = recommended hip-collapse velocity range: 0.0 - 1.0 m/sec 
% example: [err, toq] = compute_collapse_torques(1, 60, [0:0.003:0.015], 0.0);    

    tic

    % Loading optimal trajectory results (Backward Euler, 50 nodes for half-gait cycle)
    load result006_BE50.mat;
    % Convert half gait-cycle data to full gait-cycle data
    displacement = Result.dur * Result.problem.speed;   % displacement in half gait cycle
    X = Result.x;
    U = Result.u;
    [Xf,Uf] = create_fullgait(X,U,displacement);
    Result.x = Xf;
    Result.u = Uf;
    
    % Loading LQR gain results
    load(['Results_QR',num2str(QR_ratio)','.mat']);
    
    % Running inverse kinematics routine to compute global segment orientations (wrt +X) after collapse and corresponding joint angle errors used for closed-loop feedback
    s = 0;
    for ti_sel = ti_in; % [20] % select time-index over trajectory (1-100, for full gait-cycle trajectory, ~1-50 is right-foot ground contact)
        for hcd_sel = hcd_in; % [0:0.006:0.018] % 0.01 0.10] % 0.03 0.05 0.10]; % hip collapse distance in 'meters'
            for hcv_sel = hcv_in; % [0.0 1.0] % hip collapse velocity in 'meters/sec'
                [gso1, gso2, err, pos1, pos2, ang1, ang2] = IK_walk_collapse(Result, ti_sel, hcd_sel, hcv_sel);
                s = s + 1;         
                DATA_set{s}.ti = ti_sel;
                DATA_set{s}.hcd = hcd_sel;
                DATA_set{s}.hcv = hcv_sel;
                DATA_set{s}.err = err;
                DATA_set{s}.gso1 = gso1;
                DATA_set{s}.gso2 = gso2;
                DATA_set{s}.pos1 = pos1;
                DATA_set{s}.pos2 = pos2;
                DATA_set{s}.ang1 = ang1;
                DATA_set{s}.ang2 = ang2;                      
                % Computing control torques for LQR feedback
                k = squeeze(Result_optfb.k(:,:,ti_sel));
                u_lqr = -k*err';
                % Computing control torques for PD jt-feedback   
                kp = max(max(abs(k(:,4:9))));
                kd = max(max(abs(k(:,13:18))));
                for jt = 1:size(k,1)
                    u_pd(jt) = -(kp*err(jt+3) + kd*err(jt+12));
                end
                % Zeroing small torques before storing
                for jt = 1:size(k,1)
                    if abs(u_lqr(jt)) < 1e-10
                        u_lqr(jt) = 0;
                    end
                    if abs(u_pd(jt)) < 1e-10
                        u_pd(jt) = 0;
                    end
                end
                DATA_set{s}.LQR_gains = k;
                DATA_set{s}.PD_gains = [kp kd];
                DATA_set{s}.LQR_torques = u_lqr;
                DATA_set{s}.PD_torques = u_pd;
                toc
            end
        end
    end    
    
    % Plotting collapse results (stick-figure angles, torques) and
    % returning joint error and torque results
    [err, toq, pos] = plot_collapse_torques(DATA_set);

end

% Inverse kinematics routine to compute global angle/orientation of model segments following prescriped collpase at hip-joint
function [gso1, gso2, err, pos1, pos2, ang1, ang2] = IK_walk_collapse(Result, ti, hcd, hcv)        

    % Length of thigh, shank
    L_thigh = 0.4410;
    L_shank = 0.4428; 

    % Torso angle (global)
    tor = Result.x(3,ti);

    % Initial right, left leg joint angles
    ang1.torso = tor;
    ang1.rhip = Result.x(4,ti);
    ang1.rkne = Result.x(5,ti);
    ang1.rank = Result.x(6,ti);
    ang1.lhip = Result.x(7,ti);
    ang1.lkne = Result.x(8,ti);
    ang1.lank = Result.x(9,ti);        

    % Segment (thigh, shank, foot) angles with respect to +X (Right) global
    % axis. Note: leg-segment vectors proximal to distal, torso vector is
    % still pointed superiorly.
    gso1.torso =  tor + pi/2; 
    gso1.rthigh = tor - pi/2 + ang1.rhip;
    gso1.rshank = tor - pi/2 + ang1.rhip + ang1.rkne;
    gso1.rfoot =  tor - pi/2 + ang1.rhip + ang1.rkne + ang1.rank + pi/2;    
    gso1.lthigh = tor - pi/2 + ang1.lhip;
    gso1.lshank = tor - pi/2 + ang1.lhip + ang1.lkne;
    gso1.lfoot =  tor - pi/2 + ang1.lhip + ang1.lkne + ang1.lank + pi/2;
    gso2 = gso1;

    % Solve for initial hip, knee, and ankle positions
    pos1.rhip = Result.x(1:2,ti)';
    pos1.rkne = pos1.rhip + [cos(gso1.rthigh)*L_thigh sin(gso1.rthigh)*L_thigh];
    pos1.rank = pos1.rkne + [cos(gso1.rshank)*L_shank sin(gso1.rshank)*L_shank];
    pos1.lhip = pos1.rhip;
    pos1.lkne = pos1.lhip + [cos(gso1.lthigh)*L_thigh sin(gso1.lthigh)*L_thigh];
    pos1.lank = pos1.lkne + [cos(gso1.lshank)*L_shank sin(gso1.lshank)*L_shank];     
    
    % Assigning ground y-coordinate according to minimum ankle position
    ground_y = min([pos1.rank(2) pos1.lank(2)]);
    
    % Assigning final ankle and hip positions following collapse according to ground contact
    pos2 = pos1;
    if (pos1.rank(2) - hcd) < ground_y
        pos2.rank(2) = ground_y;
    else
        pos2.rank(2) = pos1.rank(2) - hcd;
    end
    if (pos1.lank(2) - hcd) < ground_y
        pos2.lank(2) = ground_y;
    else
        pos2.lank(2) = pos1.lank(2) - hcd;
    end
    pos2.rhip(2) = pos2.rhip(2) - hcd;
    pos2.lhip(2) = pos2.lhip(2) - hcd;

    % Solving for segment orientations following prescribed collapse of hip
    options = optimoptions('fmincon','Algorithm','interior-point','Diagnostics','off'); % run interior-point algorithm
    % Right Leg
    a = [pos2.rhip(1) pos2.rhip(2) pos2.rank(1) pos2.rank(2) L_thigh, L_shank];
    xo = [gso1.rthigh gso1.rshank];
    x = fmincon(@(x) myfun(x,a),xo,[],[],[],[],[],[],@(x) mycon(x,a),options); % solving for "new" global orientations of thigh and shank
    gso2.rthigh = x(1);
    gso2.rshank = x(2);
    % Left Leg
    a = [pos2.lhip(1) pos2.lhip(2) pos2.lank(1) pos2.lank(2) L_thigh, L_shank];
    xo = [gso1.lthigh gso1.lshank];
    x = fmincon(@(x) myfun(x,a),xo,[],[],[],[],[],[],@(x) mycon(x,a),options); % solving for "new" global orientations of thigh and shank
    gso2.lthigh = x(1);
    gso2.lshank = x(2);
       
    % Assigning final knee positions
    pos2.rkne = pos2.rhip + [cos(gso2.rthigh)*L_thigh sin(gso2.rthigh)*L_thigh];
    pos2.lkne = pos2.lhip + [cos(gso2.lthigh)*L_thigh sin(gso2.lthigh)*L_thigh];
    
    % Final right, left joint angles
    ang2.rhip = gso2.rthigh - gso2.torso + pi;
    ang2.rkne = gso2.rshank - gso2.rthigh;
    ang2.rank = gso2.rfoot  - gso2.rshank - pi/2;    
    ang2.lhip = gso2.lthigh - gso2.torso + pi;
    ang2.lkne = gso2.lshank - gso2.lthigh;
    ang2.lank = gso2.lfoot  - gso2.lshank - pi/2;
    
    % Computing state-feedback errors (18)
    err = zeros(1,18); 
    err(2) = pos2.rhip(2) - pos1.rhip(2);   % hip-y error
    err(4) = ang2.rhip  - ang1.rhip;        % rhip angle error   
    err(5) = ang2.rkne - ang1.rkne;         % rkne angle error
    err(6) = ang2.rank - ang1.rank;         % rank angle error
    err(7) = ang2.lhip  - ang1.lhip;        % rhip angle error   
    err(8) = ang2.lkne - ang1.lkne;         % rkne angle error
    err(9) = ang2.lank - ang1.lank;         % rank angle error 
    err(11) = -hcv;                         % hip-y velocity error
    % Specifying joint derivative errors according to hip collapse distance and velocity
    if (hcv>0)&(hcd>0)       
        tc = err(2)/err(11); % collapse time
        for dof = 4:9
            err(dof+9) = err(dof)/tc; % estimating joint velocity error by assuming proportional to joint error by collapse time
        end
    end
    
end

% Objective function, minimizing distance of thigh distal end-point and shank proximal end-point
function f = myfun(x,a)
    pos1 = [a(1) a(2)];
    pos2 = [a(3) a(4)];
    L1 = a(5);
    L2 = a(6);
    pos1 = pos1 + [cos(x(1))*L1 sin(x(1))*L1];
    pos2 = pos2 - [cos(x(2))*L2 sin(x(2))*L2];
    f = calc_dist(pos1, pos2);
end

% Constraint function to preent knee hyper-extension
function [c,ceq] = mycon(x,a)
    pos1 = [a(1) a(2)];
    pos2 = [a(3) a(4)];
    L1 = a(5);
    L2 = a(6);
    c = x(2) - x(1);          
    pos3a = pos1 + [cos(x(1))*L1 sin(x(1))*L1];
    pos3b = pos2 - [cos(x(2))*L2 sin(x(2))*L2];
    ceq(1) = calc_dist(pos1, pos3b) - L1;
    ceq(2) = calc_dist(pos2, pos3a) - L2;
end

% Sub-function to compute distance between points p and q
function d = calc_dist(p, q)
    d = sqrt((p(1) - q(1))^2 + (p(2) - q(2))^2);
end

% Creating full gait-cycle data set from half gait-cycle data and assuming right-left symmetry
function [Xf,Uf] = create_fullgait(X,U,displacement)

    nstates = size(X,1); % Number of states
    ncontrols = size(U,1); % Number of controls
    N = size(X,2); % Number of time-nodes

    % Initializing full gait-cycle size and first half gait-cycle data for X, U
    Xf = zeros(nstates,2*N); 
    Uf = zeros(ncontrols,2*N); 
    Xf(:,1:N) = X;
    Uf(:,1:N) = U;

    % Filling-out second half gait-cycle data for X, A, and B based on right-left leg symmetry
	sis = [1:3 7:9 4:6 10:12 16:18 13:15]; % symmetry index parameters for states
	sic = [1:3 7:9 4:6]; % symmetry index parameters for controls 
    Xf(:,(N+1):2*N) = X(sis,1:N); 
    Xf(1,(N+1):2*N) = X(1,1:N) + displacement; % Adding global x progress from first half gait-cycle
    Uf(:,(N+1):2*N) = U(sic,1:N);

end

% Plotting results of collapse profiles
function [err, toq, pos] = plot_collapse_torques(DATA_set)    
    
    for n = 1:length(DATA_set);
        d = DATA_set{n};
        for k = 1:2            
            rt_leg1(k,:) = [d.pos1.rhip(k) d.pos1.rkne(k) d.pos1.rank(k)];
            lt_leg1(k,:) = [d.pos1.lhip(k) d.pos1.lkne(k) d.pos1.lank(k)];        
            rt_leg2(k,:) = [d.pos2.rhip(k) d.pos2.rkne(k) d.pos2.rank(k)];
            lt_leg2(k,:) = [d.pos2.lhip(k) d.pos2.lkne(k) d.pos2.lank(k)];
        end    
        % Plotting collapse stick-figures
        figure(1);
        px1 = plot(rt_leg1(1,:), rt_leg1(2,:),'bo-'); hold on; axis equal;
        px2 = plot(lt_leg1(1,:), lt_leg1(2,:),'go-');
        set(px1, 'linewidth', 6);
        set(px2, 'linewidth', 6);
        px1 = plot(rt_leg2(1,:), rt_leg2(2,:),'bo:');
        px2 = plot(lt_leg2(1,:), lt_leg2(2,:),'go:');
        set(px1, 'linewidth', 1);
        set(px2, 'linewidth', 1);
        % Collecting collapse joint-torque data for subsequent plotting
        figure(2);
        u_lqr(n,:) = d.LQR_torques;
        u_pd(n,:) = d.PD_torques;
        jt_err(n,:) = d.err(4:9);
        hcd(n) = d.hcd;
        hcv(n) = d.hcv;
        % Storing leg position data
        pos{n}.rt_leg1 = rt_leg1;
        pos{n}.lt_leg1 = lt_leg1;
        pos{n}.rt_leg2 = rt_leg2;
        pos{n}.lt_leg2 = lt_leg2;
    end
    figure(1); legend('right', 'left');
    
    % Plotting collapse joint-torques versus joint-errors
    figure(2);
    jt_id = [1 4 2 5 3 6];
    jt_label{1} = 'rt-hip (flexion)';
    jt_label{2} = 'rt-kne (extension)';
    jt_label{3} = 'rt-ank (dorsi-flexion)';
    jt_label{4} = 'lt-hip (flexion)';
    jt_label{5} = 'lt-kne (extension)';
    jt_label{6} = 'lt-ank (dorsi-flexion)';
    for jt = 1:size(jt_err,2)
        subplot(3,2,jt);
        px1 = plot(jt_err(:,jt_id(jt)), u_pd(:,jt_id(jt)), 'r*-'); hold on;
        px2 = plot(jt_err(:,jt_id(jt)), u_lqr(:,jt_id(jt)), 'ko-');                
        set(px1, 'linewidth', 2);
        set(px2, 'linewidth', 2);        
        xlabel('jt-err(rad)');
        ylabel('torque(N-m)');
        title(jt_label{jt_id(jt)});
    end
    legend('PD', 'LQR');
    
    % Plotting collapse joint-torques versus hip collapse displacement
    figure(3);
    jt_id = [1 4 2 5 3 6];
    jt_label{1} = 'rt-hip (flexion)';
    jt_label{2} = 'rt-kne (extension)';
    jt_label{3} = 'rt-ank (dorsi-flexion)';
    jt_label{4} = 'lt-hip (flexion)';
    jt_label{5} = 'lt-kne (extension)';
    jt_label{6} = 'lt-ank (dorsi-flexion)';
    for jt = 1:size(jt_err,2)
        subplot(3,2,jt);
        px1 = plot(hcd, u_pd(:,jt_id(jt)), 'r*-'); hold on;
        px2 = plot(hcd, u_lqr(:,jt_id(jt)), 'ko-');
        set(px1, 'linewidth', 2);
        set(px2, 'linewidth', 2); 
        xlabel('hip collapse distance (m)');
        ylabel('torque(N-m)');
        title(jt_label{jt_id(jt)});
    end
    legend('PD', 'LQR');
    
    % Returning torque, error data
    err.jt_ang = jt_err;
    err.hcd = hcd;
    err.hcv = hcv;
    toq.lqr = u_lqr;
    toq.pd = u_pd;  

end