% PROGRAM DESCRIPTION: Plotting results from 2-D walking simulations
% that include LQR closed-loop feedback during simulation loop to identify
% controller performance characteristics
% by Raviraj Nataraj, 20150414
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

function PLOT_walksim_results(ctrl_opt, pf_mag, pf_opt, pf_dir, QR_ratio, n, xp_opt, plot_opt)
% ctrl_opt = 'lqr','pd' 
% pf_mag = 1,2,5,7,10 (N) 
% pf_opt = 1(growing-mag),2(bounded-mag) 
% pf_dir = 'forward','bidir'
% QR_ratio = 1e-3,1e-2,1e-1,2e-1,5e-1,1e0,2e0,5e0,1e1,2e1,5e1,1e2,1e3,1e4
% xp_opt: 1 = time, 2 = gait-cycle%, 3 = #nodes
% plot_opt = 1 = STATES, 2 = CONTROLS, 3 = OPTIMAL FEEDBACK GAINS, 
% 4 = PERTURBATION, 5 = LQR CONTROLS, 6 = LQR STATE CONTRIBUTIONS TO JOINT TORQUES, 7 = STATE-ERRORS   

    close all;
    
    nodes = 50;
    
    % Loading select (#nodes,Q/R,sim#) simulation walking results (i.e., Result_walksim structure)
    load(['walksim results\BE',num2str(nodes),'_',ctrl_opt,'_pert',num2str(pf_mag),'_opt',num2str(pf_opt),'_',pf_dir,'\Results_QR',num2str(QR_ratio),'_n',num2str(n),'.mat'], 'Result_walksim', 'ctrl_chars_sim'); 

    % Distributing walking simulation results, parameters to variables
    res = Result_walksim;
    U_open = res.U_open; U_lqr = res.U_lqr; U_app = res.U_app;
    X = res.X; X_des = res.X_des; Err = X - X_des; K = res.K;
    pert_force = res.pert_force;
    nstates = res.problem.nstates; ncontrols = res.problem.ncontrols;
    
    % Plotting variables with x-axis as time (sec) or gait-cycle%    
    time = res.t;
    gait_cycle = 100*(res.t)/res.problem.cyc_dur;
    switch(xp_opt) %xp_opt: 1 = time, 2 = gait-cycle%, 3 = #nodes
        case{1}
            xp = time; xlabel_sel = 'TIME (sec)';
        case{2}
            xp = gait_cycle; xlabel_sel = 'GAIT-CYCLE%';
        case{3}
            xp = 1:length(time); xlabel_sel = '#TIME-NODES';
    end    

    % Assigning state, joint names and units
    st_name{1} = 'HIP-X'; st_unit{1} = 'm';
    st_name{2} = 'HIP-Y'; st_unit{2} = 'm';
    st_name{3} = 'TORSO-TILT'; st_unit{3} = 'rad';
    st_name{4} = 'RT HIP-ANG'; st_unit{4} = 'rad';
    st_name{5} = 'RT KNE-ANG'; st_unit{5} = 'rad';
    st_name{6} = 'RT ANK-ANG'; st_unit{6} = 'rad';
    st_name{7} = 'LT HIP-ANG'; st_unit{7} = 'rad';
    st_name{8} = 'LT KNE-ANG'; st_unit{8} = 'rad';
    st_name{9} = 'LT ANK-ANG'; st_unit{9} = 'rad';
    st_name{10} = 'HIP-X DERIV'; st_unit{10} = 'm/s';
    st_name{11} = 'HIP-Y DERIV'; st_unit{11} = 'm/s';
    st_name{12} = 'TORSO-TILT DERIV'; st_unit{12} = 'rad/s';
    st_name{13} = 'RT HIP-ANG DERIV'; st_unit{13} = 'rad/s';
    st_name{14} = 'RT KNE-ANG DERIV'; st_unit{14} = 'rad/s';
    st_name{15} = 'RT ANK-ANG DERIV'; st_unit{15} = 'rad/s';
    st_name{16} = 'LT HIP-ANG DERIV'; st_unit{16} = 'rad/s';
    st_name{17} = 'LT KNE-ANG DERIV'; st_unit{17} = 'rad/s';
    st_name{18} = 'LT ANK-ANG DERIV'; st_unit{18} = 'rad/s';
    jt_name{1} = 'Rt HIP'; jt_unit{1} = 'N-m';
    jt_name{2} = 'Rt KNEE'; jt_unit{2} = 'N-m';
    jt_name{3} = 'Rt ANKLE'; jt_unit{3} = 'N-m';
    jt_name{4} = 'Lt HIP'; jt_unit{4} = 'N-m';
    jt_name{5} = 'Lt KNEE'; jt_unit{5} = 'N-m';
    jt_name{6} = 'Lt ANKLE'; jt_unit{6} = 'N-m';
    
    % Plotting option-select walking simulation results    
    switch(plot_opt)
        case{1} % plotting STATES
            figure(1); 
            for i = 1:nstates
                sx = subplot(6,3,i);
                px1 = plot(xp, X_des(:,i),'b'); hold on;
                px2 = plot(xp, X(:,i),'g:');
                tx = title(['st#',num2str(i), '-->',st_name{i}]);
                set(px1,'linewidth',3);
                set(px2,'linewidth',3);
                set(sx,'fontweight','bold','fontsize',12);
                set(tx,'fontweight','bold','fontsize',14);
                ax = xlabel(xlabel_sel); set(ax,'fontweight','bold','fontsize',8);
                bx = ylabel(st_unit{i}); set(bx,'fontweight','bold','fontsize',8);
            end
            legend('desired/optimal state','actual');

        case{2} % plotting CONTROLS
            figure(2);
            for i = 1:ncontrols
                sx = subplot(2,3,i);
                px1 = plot(xp, U_open(:,i),'b'); hold on;
                px2 = plot(xp, U_app(:,i),'r:');
                tx = title(['control#',num2str(i), '-->',jt_name{i}]);
                set(px1,'linewidth',3);
                set(px2,'linewidth',3);
                set(sx,'fontweight','bold','fontsize',12);
                set(tx,'fontweight','bold','fontsize',14);
                ax = xlabel(xlabel_sel); set(ax,'fontweight','bold','fontsize',8);
                bx = ylabel(jt_unit{i}); set(bx,'fontweight','bold','fontsize',8);
            end
            legend('desired/optimal control','actual');

        case{3} % plotting OPTIMAL FEEDBACK GAINS
            figure(3);
            for i = 1:nstates
                sx = subplot(6,3,i);
                px = plot(xp, squeeze(K(:,:,i)));
                tx = title(['st#',num2str(i), '-->',st_name{i}]);
                set(px,'linewidth',1.5);
                set(sx,'fontweight','bold','fontsize',12);
                set(tx,'fontweight','bold','fontsize',14);
                ax = xlabel(xlabel_sel); set(ax,'fontweight','bold','fontsize',8);
                bx = ylabel([jt_unit{1},'/state-unit']); set(bx,'fontweight','bold','fontsize',8);
            end
            for i = 1:ncontrols
                jt_name_gain{i} = [jt_name{i},'-gain'];
            end
            legend(jt_name_gain);

        case{4} % plotting PERTURBATION
            figure(4);
            sx = subplot(1,1,1);
            px = plot(xp, pert_force);
            tx = title('random perturbation force --> +hip-x');
            set(px,'linewidth',1.5);
            set(sx,'fontweight','bold','fontsize',12);
            set(tx,'fontweight','bold','fontsize',14);
            ax = xlabel(xlabel_sel); set(ax,'fontweight','bold','fontsize',8);
            bx = ylabel('force (N)'); set(bx,'fontweight','bold','fontsize',8);

        case{5} % plotting LQR CONTROLS            
            figure(5);
            for i = 1:ncontrols
                sx = subplot(2,3,i);
                px1 = plot(xp, 0*U_open(:,i),'b'); hold on;
                px2 = plot(xp, U_lqr(:,i),'r:');
                tx = title(['control#',num2str(i), '-->',jt_name{i}]);
                set(px1,'linewidth',3);
                set(px2,'linewidth',3);
                set(sx,'fontweight','bold','fontsize',12);
                set(tx,'fontweight','bold','fontsize',14);
                ax = xlabel(xlabel_sel); set(ax,'fontweight','bold','fontsize',8);
                bx = ylabel(['RMS = ',num2str(rms(U_lqr(:,i)),'%6.2f'),' ',jt_unit{i}]); set(bx,'fontweight','bold','fontsize',12);
            end
            legend('zero control',['LQR control']);

        case{6} % plotting LQR JOINT-TORQUE CONTRIBUTIONS FROM EACH FEEDBACK STATE 
            for i = 1:nstates
                k = squeeze(K(:,:,i));
                for j = 1:ncontrols
                    torques(:,i,j) = -k(:,j).*(X(:,i)-X_des(:,i));
                end
            end
            for jt = 1:6
                figure(60+jt)
                for st = 1:18
                    sx = subplot(7,3,st);
                    px = plot(xp, squeeze(torques(:,st,jt)));
                    tx = title(['jt#',num2str(jt),', st#',num2str(st), '-->',st_name{st}]);
                    set(px,'linewidth',1.5);
                    set(sx,'fontweight','bold','fontsize',12);
                    set(tx,'fontweight','bold','fontsize',14);
                end                
                sx = subplot(7,1,7);
                px = plot(xp, sum(squeeze(torques(:,:,jt)')));
                tx = title(['total LQR torque at ', jt_name{jt}]);
                ax = xlabel(xlabel_sel); set(ax,'fontweight','bold','fontsize',8);
                bx = ylabel(jt_unit{jt}); set(bx,'fontweight','bold','fontsize',8);
            end
            
        case{7} % plotting STATE-ERRORS
            figure(1);
            for i = 1:nstates
                sx = subplot(6,3,i);
                px1 = plot(xp, Err(:,i),'R'); hold on;
                tx = title(['st#',num2str(i), '-->',st_name{i}]);
                set(px1,'linewidth',3);
                set(sx,'fontweight','bold','fontsize',12);
                set(tx,'fontweight','bold','fontsize',14);
                ax = xlabel(xlabel_sel); set(ax,'fontweight','bold','fontsize',8);
                bx = ylabel(st_unit{i}); set(bx,'fontweight','bold','fontsize',8);
            end
            legend('state error');
            
    end

end