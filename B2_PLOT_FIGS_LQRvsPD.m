% PROGRAM DESCRIPTION: Comparing results between LQR and PD closed-loop
% control for walking simulations across Q/R ratios for different
% perturbation cases (pert mag, pert direction)
% by Raviraj Nataraj, 20150529
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

function B2_PLOT_FIGS_LQRvsPD

tic
close all;

ctrl_opt_plot = 2; % 1 = PD, 2 = LQR for figures (pert_case) 1 through 4

for sbp = 2:-1:1
    for pert_case = 1:3 % pert_case = 1 (Time-to-Fall), 2 (Closed-Loop Torque RMS), 3 (Total Torque RMS), 4 (Mean Gain Value)
        
        % Figure, sub-plot, data loading for each controller option, and perturbation case
        figure(pert_case);
        sx = subplot(1,2,sbp);
        title_label{1} = 'All Tested Controllers';
        title_label{2} = 'Successful Controllers Only';
        ctrl_opt_label{1} = 'pd';
        ctrl_opt_label{2} = 'lqr';
        for ctrl_opt = 1:2
            switch(pert_case)
                case{1}
                    pert_dir = 'forward'; pert_mag_sel = 10; pf_opt = 1;
                    leg_sel{1} = [ctrl_opt_label{ctrl_opt}, '-control, perturbation: 10N/sec growing - forward'];
                    sx = subplot(1,1,1);
                case{2}
                    pert_dir = 'bidir'; pert_mag_sel = [5]; pf_opt = 2;
                    leg_sel{1} = [ctrl_opt_label{ctrl_opt}, '-control, perturbation: 5N bounded - bidirectional'];
                case{3}
                    pert_dir = 'bidir'; pert_mag_sel = [5]; pf_opt = 2;
                    leg_sel{1} = [ctrl_opt_label{ctrl_opt}, '-control, perturbation: 5N bounded - bidirectional'];
                case{4}
                    pert_dir = 'bidir'; pert_mag_sel = [5]; pf_opt = 2;
                    leg_sel{1} = [ctrl_opt_label{ctrl_opt}, '-control'];
                    sx = subplot(1,1,1);
            end
            
            
            % Plotting onto sub-plot results for each select perturbation magnitude
            p = 0; pc = [{'b'},{'g'},{'r'},{'c'},{'m'}];
            for pert_mag = pert_mag_sel
                
                % Loading select controller case data file
                load(['DATA_',ctrl_opt_label{ctrl_opt},'_pert',num2str(pert_mag),'_opt',num2str(pf_opt),'_', pert_dir,'.mat']);
                
                % Assigning composite data to variables
                c = ctrl_chars_comp;
                QR_ratio = c(:,1);
                sim_num = c(:,2);
                I_fall = c(:,3);
                T_fall = c(:,4);
                P_mean = c(:,5);
                C_xport = c(:,6);
                K_mean = c(:,7);
                W_fall = c(:,8);
                E_fall = c(:,9);
                P_mean2 = c(:,10);
                P_int = c(:,11);
                P_int2 = c(:,12);
                P_mean3 = c(:,13);
                PD_mean = c(:,14);
                P_mean4 = c(:,15);
                n_sim = 20;
                
                % Data variable of interest for particular perturbation control case
                switch(pert_case)
                    case{1}
                        data_var = T_fall; ylabel_name = 'Time-to-Fall (Sec)'; ylim_sel = [0 12]; title_name = title_label{sbp};
                    case{2}
                        data_var = P_mean3; ylabel_name = 'Closed-Loop Torque RMS (N-m)'; ylim_sel = [20 120]; title_name = title_label{sbp};
                    case{3}
                        data_var = P_mean4; ylabel_name = 'Total Torque RMS (N-m)'; ylim_sel = [20 120]; title_name = title_label{sbp};
                    case{4}
                        if ctrl_opt_plot == 1
                            data_var = PD_mean; ylabel_name = 'Mean PD Gain Value'; ylim_sel = [20 120];
                        elseif ctrl_opt_plot == 2
                            data_var = K_mean; ylabel_name = 'Mean LQR Gain Value'; ylim_sel = [20 120];
                        end
                end
                
                % Computing mean+/- performance data versus Q/R
                p = p + 1;
                c = ctrl_opt;
                n_QR = length(QR_ratio)/n_sim;
                for i = 1:n_QR
                    p1 = (i-1)*n_sim + 1; p2 = p1 + n_sim - 1;
                    QR{p}(i) = mean(QR_ratio(p1:p2));
                    m{p}(i) = mean(data_var(p1:p2));
                    q{p}(i) = std(data_var(p1:p2));
                    K{p}(i) = mean(K_mean(p1:p2));
                end
                
            end
            
            % Plotting mean +/-s.d. performance data versus Q/R
            n_ctrl = length(QR{1});
            n_sim = length(P_mean3)/n_ctrl;
            ctrl_index = 1:n_ctrl;
            if (sbp == 2)&(pert_case~=1)&(pert_case~=4)
                ctrl_index = [3:n_ctrl]; % removing first two low-gain (failed) controllers
            end
            if ctrl_opt == ctrl_opt_plot % LQR
                np = p;
                for p = 1:np
                    if pert_case == 4
                        px = plot(QR{p}(ctrl_index), m{p}(ctrl_index), 'b*-'); set(px, 'linewidth', 2); set(gca,'xscale','log');
                    else
                        ex = errorbar(QR{p}(ctrl_index), m{p}(ctrl_index), q{p}(ctrl_index), 'b*-'); set(ex, 'linewidth', 2); set(gca,'xscale','log');
                        tx = title(title_name); set(tx, 'fontweight', 'bold', 'fontsize', 18);
                    end
                    ax = xlabel('Q/R'); bx = ylabel(ylabel_name);
                    set(ax, 'fontweight', 'bold', 'fontsize', 15); set(bx, 'fontweight', 'bold', 'fontsize', 15); set(sx, 'fontweight', 'bold', 'fontsize', 15);
                end
                lx = legend(leg_sel);%, 'Location','NorthEast');
                set(lx, 'fontweight', 'bold', 'fontsize', 12);
            end
            
            % Storing data for RMS versus Time-to-Fall plots
            switch(pert_case)
                case{1}
                    data_Tfall{ctrl_opt} = T_fall;
                case{2}
                    data_Prms{ctrl_opt} = P_mean3;
            end
            
        end
    end
    
    % Plotting RMS vs Time-to-Fall results
    figure(pert_case+1);
    sxp = subplot(2,1,sbp);
    clear pc; pc{1} = 'b'; pc{2} = 'g';
    if (sbp == 2)
        ctrl_index = [3:n_ctrl]; % removing first two low-gain controllers
    end
    for c = ctrl_index
        lw = 4*(c/n_ctrl); % line-width
        % plotting means
        for ctrl = 1:2        
            p1 = (c-1)*n_sim + 1; p2 = p1 + 19;
            mx = mean(data_Prms{ctrl}(p1:p2));
            my = mean(data_Tfall{ctrl}(p1:p2));            
            px = plot(mx,my,[pc{ctrl},'o']); hold on;
            set(px, 'linewidth', lw*0.5);
        end
        lx = legend('PD', 'LQR');
        % plotting std error-bars
        for ctrl = 1:2
            mx = mean(data_Prms{ctrl}(p1:p2));
            my = mean(data_Tfall{ctrl}(p1:p2));
            sx = std(data_Prms{ctrl}(p1:p2));            
            sy = std(data_Tfall{ctrl}(p1:p2));                    
            my_errorbar(mx, my, sx, sy, pc{ctrl},lw);
        end
    end
    ax = xlabel('Closed-Loop Torque RMS (N-m)');
    bx = ylabel('Time-to-Fall (Sec)');    
    set(lx, 'fontweight', 'bold', 'fontsize', 12);
    set(ax, 'fontweight', 'bold', 'fontsize', 15);
    set(bx, 'fontweight', 'bold', 'fontsize', 15);
    set(sxp, 'fontsize', 15, 'fontweight', 'bold');
    tx = title(title_name); set(tx, 'fontweight', 'bold', 'fontsize', 18);
end

end