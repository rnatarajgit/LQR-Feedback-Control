% PROGRAM DESCRIPTION: Script to run batch simulations to identify 
% performance characteristics of optimal feedback (LQR) controller for 
% torque control of 2-D walking model (gait2dem). Performance 
% characteristics include time-to-fall, mean joint power, and cost of 
% transport while modulating single control parameter (Q/R ratio, i.e., 
% tracking/effort).
% by Raviraj Nataraj, 20150413
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

function A2_LQR_batch_walksim
   
    clear;
    
    tic
    
    global nodes QR_ratio n                 % global variables declared for screen output in sub-function 'odefun'
    
    for nodes = 50; % [50 100 200 400];     % number of nodes in half gait-cycle, data sets collected/outputted in respective folders (Backward Euler, N = 50, 100, 200, 400)
        opt.n_sim = 20;                     % number of simulations to run for each controller
        opt.n_cyc = 10;                     % number of gait-cycle to run each simulation
        opt.ctrl_opt = 2;                   % controller-mode option (1 = open-loop torques, 2 = PD feedack, 3 = LQR feedback)
        opt.pf_mag = 5;                     % perturbation magnitude (pfopt = 1, growth-rate magnitude (N/s); pfopt = 2, constant magnitude (N); 
        opt.pf_opt = 2;                     % random perturbation type (pfopt = 1, growing-magnitude, pfopt = 2, constant-magnitude)
        opt.pf_int = 0.1;                   % perturbation force interval of change (in sec)
        opt.QR_ratio_sel = [1e-3 1e-2]; %[1e-3 1e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0 1e1 2e1 5e1 1e2 1e3 1e4];     % controller design parameter (Q/R ratio)
        
        ctrl_chars_comp = [];    % initializing structure to store composite loop-simulation controller characteristics
        for QR_ratio =  opt.QR_ratio_sel
            
            % Loading individual optimal feedback controller results mat-file corresponding to specific QR_ratio
            load(['optim feedback gains\BE',num2str(nodes),'\Results_QR',num2str(QR_ratio),'.mat']);
           
            % Running individual walking simulations
            for n = 1:opt.n_sim                
                [Result_walksim] = walksim(Result_optfb, opt);                                                                                                      % running controller walking simulation with user option inputs: optimal feedback results, controller option, #cycles to run, perturbation type, perturbation rate
                [I_fall, T_fall, P_mean, C_xport, K_mean, W_fall, E_fall, P_mean2, P_int, P_int2, P_mean3, PD_mean, P_mean4] = compute_chars(Result_walksim);       % computing controller characteristics from simulation
                ctrl_chars_sim = [QR_ratio, n, I_fall, T_fall, P_mean, C_xport, K_mean, W_fall, E_fall, P_mean2, P_int, P_int2, P_mean3, PD_mean, P_mean4];         % controller chracteristics of each simulation
                ctrl_chars_comp = [ctrl_chars_comp; ctrl_chars_sim];                                                                                                % concatenating controller characteristics of each simulation for storage
                %=========================================================%
                % Saving individual walking simulation results to mat-file
                ctrlopt_label{2} = 'pd';
                ctrlopt_label{3} = 'lqr';
                pfdir_label{1} = 'forward';
                pfdir_label{2} = 'bidir';
                save(['walksim results\BE',num2str(nodes),'_',ctrlopt_label{opt.ctrl_opt},'_pert',num2str(opt.pf_mag),'_opt',num2str(opt.pf_opt),'_',pfdir_label{opt.pf_opt},'\Results_QR',num2str(QR_ratio),'_n',num2str(n),'.mat'], 'Result_walksim', 'ctrl_chars_sim');   
                %=========================================================%
            end
            clear Res_optfb;
            
        end
        
        save Results_composite ctrl_chars_comp;         % Saving composite controller characteristic results
    end

    toc
end
%=============================================================================================
% Running forward simulation of 2-D walking model with user options for control, number of gait cycles, and magnitude of growing perturbation 
function [Result_walksim] = walksim(Result_optfb, opt) 

    % Declaring Results structure as global variable to be passed to 'odefun', 'controller' sub-functions
	global Res
    Res = [];
                 
    % Assigning select constants from Results structure
    nstates = Result_optfb.problem.nstates;             % number of states
    ncontrols = Result_optfb.problem.ncontrols;         % number of nodes
    speed = Result_optfb.problem.speed;                 % walking speed in m/sec
    cyc_dur = Result_optfb.problem.dur;                 % single gait-cycle duration in seconds
    N = Result_optfb.problem.N;                         % number of time-nodes
       
    % Initialize time-samples
    outputinterval = cyc_dur / N;                       % time intervals
    tsamples = [0:outputinterval:cyc_dur]';             % time grid for output over single cycle        
    duration = opt.n_cyc*cyc_dur;                       % total simulation duration pending total number of cycles     
	tsamples_new = [0:outputinterval:duration]';        % time grid for output adjusted across total number of cycles
            
    % Initialize trajectory samples for x-desired, u, and k over single cycle
    sic = [1:3 7:9 4:6];                                % control symmetry index
    xsamples_des = Result_optfb.x';                     % samples for desired state trajectories
    xsamples_des(N+1,:) = xsamples_des(1,:);            % add one state sample from the same leg
    xsamples_des(N+1,1) = xsamples_des(1,1) + cyc_dur * speed; % apply bias for hip-x progression 
    usamples = Result_optfb.u(4:9,:)'; ncontrols = ncontrols - 3; sic = sic(4:9) - 3; % retaining only 6 joint torques for control samples
    usamples(N+1,:) = [usamples(1,:)];                  % add one control sample from the same leg
    ksamples = permute(Result_optfb.k,[3 1 2]);         % samples for feedback gains, permutation done to convert first column to represent time-node index     
    ksamples(N+1,:,:) = ksamples(1,:,:);                % add one feedback sample from the same leg   
        
    % Specifying perturbation loading
    Res.pert.pf_opt = opt.pf_opt;                                                   % storing user option for perturbation force loading
    Res.pert.pf_mag = opt.pf_mag;                                                   % storing value of growth_rate-magnitude (pf_opt = 1) or constant-magnitude (pf_opt = 2) of random forward perturbation (N/s)
    Res.pert.pertinterval = opt.pf_int;                                             % storing user specification of time interval of random perturbation change
    max_pert_time = ceil(duration/opt.pf_int)*opt.pf_int;                           % maximum perturbation time must be > simulation duration to ensure linear interpolation for perturbation can be done for any simulation time (else NaN's appear at end of simulation)
    Res.pert.t = 0:Res.pert.pertinterval:max_pert_time;                             % specifying perturbation time-vector over entire simualtion duration
    if opt.pf_opt == 1
        Res.pert.force = Res.pert.pf_mag*Res.pert.t.*(2*(rand(size(Res.pert.t))-0.5));        % specifying growing-magnitude random perturbation force
    elseif opt.pf_opt == 2
        Res.pert.force = Res.pert.pf_mag*(2*(rand(size(Res.pert.t))-0.5));                    % specifying constant-magnitude random perturbation force
    else
        error('pf_opt must equal 1 or 2')
    end
    
    % Saving select problem parameters and data samples to Results structure prior to running walking simulation
    Res.problem.ctrl_opt = opt.ctrl_opt;    % user option for walking simulation 
    Res.problem.speed = speed;              % constraint walking speed for optimal open-loop trajectories
    Res.problem.cyc_dur = cyc_dur;          % duration for single full gait-cycle
    Res.problem.nstates = nstates;          % number of states
    Res.problem.ncontrols = ncontrols;      % number of controls
    Res.problem.n_cyc = opt.n_cyc;          % number of gait-cycles run during simulation 
    Res.problem.dur = duration;				% total simulation duration in seconds
    Res.problem.N = opt.n_cyc*N;            % total number of simulation time-nodes
    Res.tsamples = tsamples;                % time samples for single cycle    
    Res.tsamples_new = tsamples_new;        % time samples across entire simulation
    Res.xsamples_des = xsamples_des;        % desired/optimal x-state samples for single cycle
    Res.usamples = usamples;                % optimal joint torque control samples for single cycle
    Res.ksamples = ksamples;                % optimal feedback samples for single cycle
    
    % Run the simulation       
    xinit = Result_optfb.x(:,1);	% Initialize x solution with first optimal state result
    options = odeset('Events',@walk_events);
    [t,x] = ode23(@odefun, tsamples_new, xinit, options);
	disp('simulation finished, creating animation...');
    
    % In case of early termination (walk_events), ensure that final x, t to be stored are same length as stored controller data from Results structure
    n = min([length(t) length(Res.time)]);    
    
    % Saving final walking simulation results         
    Res.x = x(1:n,:);
    Res.t = t(1:n,:);    
    Result_walksim = Res;                   % re-naming results structure for walking simulation prior to saving    
    
end
%=====================================================================================
% Ordinary differentional equation solver of states (x) of gait dynamics over time (t)
function [xdot] = odefun(t,x)    
    
    global QR_ratio n nodes Res
    
    % Specifying perturbation force by interpolcation
    pert_force = interp1(Res.pert.t,Res.pert.force,t);
    
    % Specifying controller forces, which include perturbation force to hip-x, and 6 joint torques from feedback)
    [u, ctrl_op] = controller(t,x);
    tau = [pert_force; 0; 0; u];   
    
    % Call to run gait dynamics
	xdot = gait2dem(x,tau);
    
    % Storing controller results and perturbation force applied at closest time-node
    index = round(interp1(Res.tsamples_new,1:length(Res.tsamples_new),t));
    Res.pert_force(index) = pert_force;  
    Res.U_open(index,:) = ctrl_op.u_open;
    Res.U_lqr(index,:) = ctrl_op.u_lqr;
    Res.U_pd(index,:) = ctrl_op.u_pd;
    Res.U_app(index,:) = u;  
    Res.X(index,:) = ctrl_op.x;
    Res.X_des(index,:) = ctrl_op.x_des;    
    Res.K(index,:,:) = ctrl_op.k;
    Res.PD(index,:) = ctrl_op.pd;
    Res.time(index,:,:) = t;
    
    % Outputting select results to screen
    sprintf('%d %g %d %f', nodes, QR_ratio, n, t)
    
end
%=====================================================================================
% Computing controls (u) to be applied upon gait dynamics
function [u, ctrl_op] = controller(t,x) 

    global Res   
   
    % Extracting data samples from Results structure
    tsamples = Res.tsamples;
    usamples = Res.usamples;
    xsamples_des = Res.xsamples_des;
    ksamples = Res.ksamples;
    
    % Identifying time point within cycle
    t_in_cyc = mod(t,Res.problem.cyc_dur);  
    
    % Controller: open-loop torques at each time-node
    u = interp1(tsamples, usamples, t_in_cyc);  % interpolating across optimal open-loop control samples
    u_open = u;                                 % assigning open-loop controls    
 
    % Controller: closed-loop torques from uniform PD-control at each time-node         
    x_des = interp1(tsamples,xsamples_des,t_in_cyc);                            % interpolating across optimal (desired) state samples
    n_jt = size(ksamples,2);
    for jt = 1:n_jt
        k_temp = squeeze(mean(abs(ksamples)));
        kp(jt) = max(max(abs(squeeze(ksamples(:,jt,4:9)))));
        kd(jt) = max(max(abs(squeeze(ksamples(:,jt,13:18)))));
    end
    kp = max(kp); kd = max(kd);
    % kp = 500; kd = 10;                                                        % minimal gains suggested by Ben Baldwin email 20150303
    u_pd = -kp.*(x(4:9)' - x_des(4:9)) - kd.*(x(13:18)' - x_des(13:18));        % assigning closed-loop controls from PD control law
        
    % Controller: closed-loop torques from LQR optimal feedback at each time-node
    k = my_interp(tsamples,ksamples,t_in_cyc);                                  % interpolating across optimal open-loop control samples
    cyc_displacement = Res.problem.cyc_dur * Res.problem.speed;                 % forward progression/displacement each cycle
    x_des(1) = x_des(1) + floor(t/Res.problem.cyc_dur)*cyc_displacement;        % adjusting desired trajectory of state#1 (hip-x) for forward progression across gait cycles
    u_lqr = (-k*(x - x_des'))';                                                 % assigned closed-oop controls from LQR closed-loop control law     
  
    % Applying controls with potential addition of closed-loop torques
    if (Res.problem.ctrl_opt == 1)
            u = u_open;
    elseif (Res.problem.ctrl_opt == 2)
            u = u_open + u_pd;
    elseif (Res.problem.ctrl_opt == 3)
            u = u_open + u_lqr;
    else
        error('ctrl_opt must equal 1, 2, or 3');
    end
    
    % Updating select "controller operation" data for subsequent storage in simulation Results structure    
    ctrl_op.u_open = u_open;
    ctrl_op.u_lqr = u_lqr;
    ctrl_op.u_pd = u_pd;
    ctrl_op.x = x;
    ctrl_op.x_des = x_des;
    ctrl_op.k = k;
    ctrl_op.pd = [kp kd];
        
    % Transposing control vector for subsequent input into 'gait2dem'
    u = u';

end
%=====================================================================================
% My function to linearly interpolate entire time-varying 2-D matrix (2nd, 3rd cols) between time-nodes (1st col)
function m = my_interp(tsamp,M,t)

    point = interp1(tsamp,1:length(tsamp),t);
    i1 = floor(point); 
    if i1 == length(tsamp)
        i2 = i1;
    else
        i2 = i1 + 1;
    end
    delta = point - i1;
    w1 = 1 - delta; w2 = 1 - w1;  
    m = squeeze(w1*M(i1,:,:) + w2*M(i2,:,:));

end
%=====================================================================================
% Computing closed-loop control simulation characteristics: time to fail (i.e., fall)  and joint-torque power, work, and total segment energy
function [I_fall, T_fall, P_mean, C_xport, K_mean, W_fall, E_fall, P_mean2, P_int, P_int2, P_mean3, PD_mean, P_mean4] = compute_chars(Result_walksim)

    % Distributing time, state, and control data from walking simulation
    clear t x u u_lqr u_pd;
    t = Result_walksim.t;
    x = Result_walksim.x;
    u = Result_walksim.U_app;
    u_lqr = Result_walksim.U_lqr;
    u_pd = Result_walksim.U_pd;
    
    % Mean absolute value of feedback gains being applied
    K_mean = mean(mean(mean(abs(Result_walksim.K)))); % mean LQR gain value across all state feedbacks and all time-points
    PD_mean = mean(mean(abs(Result_walksim.PD))); % mean PD gain value across all state feedbacks and all time-points

    % Solving for time to fall from hip-y coordinate exceeding threshold (i.e., 20 s.d. from mean value defined from optimal walking cycle)
    hip_y = x(:,2);                 % hip-y trajectory
    hy_mean = 0.9114;               % mean hip-y value during optimal gait-cycle performance ('result006.mat') 
    hy_std = 0.0130;                % std hip-y value during optimal gait-cycle performance ('result006.mat') 
    m = hy_mean*ones(size(hip_y));  % mean hip-y constant signal
    thresh = 20*hy_std;             % hip-y deviation threshold for fall
    s = thresh*ones(size(hip_y));   % std hip-y constant signal
    p = abs(hip_y - hy_mean);       % absolute deviations from hip-y from mean value
    q = abs(p - s);                 % absolute deviations from hip-y nearest threshold

    % Determining index when walking "fall" occurs
    if max(q) > thresh
        [~,Isort] = sort(q);        % ascend-sort absolute deviations of hip-y from threshold
        I_fall = min(Isort(1:10));  % select earliest deviation among 10 nearest deviations to threshold  
    else
        I_fall = length(t);         % in case where no hip-y deviations exceeded threshold, simply assign fall index to last time-point of simulation
    end
    T_fall = t(I_fall);             % time (in sec) when fall occurs

    % Solving for total joint-Work, mean joint-Power, total transport-Cost, and mean segmental kinetic rotational Energy to "fall" (reference - http://en.wikipedia.org/wiki/Torque)
    % Computing Work    
    n = length(x);
    dtheta = x([2:n 1],4:9) - x(1:n,4:9);
    torque = (u([2:n 1],:) + u(1:n,:))/2;
    W = sum(torque(1:I_fall,:).*dtheta(1:I_fall,:)); % Work at each joint
    W_fall = sum(W); % Total work across all joints
    % Computing Power
    w = x(:,13:18); % angular velocity
    P = torque(1:I_fall,:).*w(1:I_fall,:); % power across each joint
    Ptot = sum(P'); % total power across all joints
    P_mean = mean(Ptot); % mean total power across all joints
    % Alternatively Computing Power (see Ton e-mail 20150506)
    p_sign = sign(Ptot); p_zero = sign(p_sign + 1);
    Pplus = Ptot.*p_zero; % total positive-power
    P_mean2 = mean(Pplus);
    dt = t(2:n) - t(1:(n-1)); dt(n) = mean(dt);
    P_int = sum(Ptot'.*dt(1:I_fall)); % coarse-integral of total power signal
    P_int2 = sum(Pplus'.*dt(1:I_fall)); % coarse-integral of total positive-power signal
    % Compute Power by RMS of closed-loop torques
    for jt = 1:size(u_lqr,2)
        u_rms(:,jt) = rms(u_lqr(:,jt));
    end
    Prms = sum(u_rms');
    P_mean3 = mean(Prms);
    % Compute Power by RMS of total (open-loop + closed-loop) torques
    for jt = 1:size(u,2)
        u_rms(:,jt) = rms(u(:,jt));
    end
    Prms = sum(u_rms');
    P_mean4 = mean(Prms);
    % Computing Energy
    inertia.trunk = 3.1777;
    inertia.thigh = 0.1522;
    inertia.shank = 0.0624;
    inertia.foot = 0.0184;
    trunk_w = x(:,12);                  % torso angular velocity
    rthigh_w = trunk_w - x(:,13);       % torso angular velocity - rhip angular velocity
    rshank_w = rthigh_w - x(:,14);      % rthigh_w - rknee angular velocity
    rfoot_w = rshank_w - x(:,15);       % rshank_w - rankle angular velocity
    lthigh_w = trunk_w - x(:,16);       % torso angular velocity - lhip angular velocity
    lshank_w = lthigh_w - x(:,17);      % lthigh_w - lknee angular velocity
    lfoot_w = lshank_w - x(:,18);       % lshank_w - lankle angular velocity
    E(:,1) = inertia.trunk*trunk_w.^2;  % energy of trunk segment
    E(:,2) = inertia.thigh*rthigh_w.^2; % energy of rt-thigh segment
    E(:,3) = inertia.shank*rshank_w.^2; % energy of rt-shank segment
    E(:,4) = inertia.foot*rfoot_w.^2;   % energy of rt-foot segment
    E(:,5) = inertia.thigh*lthigh_w.^2; % energy of lt-thigh segment
    E(:,6) = inertia.shank*lshank_w.^2; % energy of lt-shank segment
    E(:,7) = inertia.foot*lfoot_w.^2;   % energy of lt-foot segment  
    E_fall = mean(mean(E(1:I_fall,:))); % mean rotational energy across all segments 
    % Computing transport cost (dimensionless)
    mass.trunk = 50.85;
    mass.thigh = 7.5;
    mass.shank = 3.4875;
    mass.foot = 1.0875;
    body_mass = mass.trunk + 2*(mass.thigh + mass.shank + mass.foot); % total-body mass
    distance = max(x(1:I_fall,1));      % forward distance traversed before fall
    C_xport = W_fall/(body_mass*distance); % transport cost to fall
            
end
%=====================================================================================
function [value,isterminal,direction] = walk_events(~,x)
    % this function defines an event that terminates the simulation (model is falling)
    hy_mean = 0.9114;               % mean hip-y value during optimal gait-cycle performance ('result006.mat') 
    hy_std = 0.0130; 
    thresh = 25*hy_std;
    if abs(x(2) - hy_mean)> thresh
        value(1) = 0;
    else 
        value(1) = 1;
    end
    %value(1) = (x(2) - height);	% simulation will stop when this becomes zero
    isterminal(1) = 1;
    direction(1) = -1;
end
%=====================================================================================