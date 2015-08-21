% PROGRAM DESCRIPTION: BATCH plotting performance curves for 1-parameter
% LQR controller from composite controller-characteristic simulation results
% by Raviraj Nataraj, 20150529
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

function B1_COMPILE_perf_curves(new_save_opt, plot_opt) 
% new_save_opt = 1, saving new data files for each controller case, else loading formerly saved results 
% plot_opt = 1 (Time-to-Fall), 2 (Closed-Loop Torque RMS) 

    close all; 
    tic
    global QR_sel sim_index n_sim pert_disp
    
    QR_sel = [1e-3 1e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0 1e1 2e1 5e1 1e2 1e3 1e4];
    sim_index = 1:20;        
           
    n_sim = length(sim_index);
    n_figcase = 0;
    pert_disp{1} = 'growing magnitude';
    pert_disp{2} = 'bounded magnitude';

    %=====================================================================%
    % FINAL RMS Result Files
    pert_opt = 2;
    pert_dir = 'bidir';
    for ctrl_opt_sel = 1:2
        switch(ctrl_opt_sel)
            case{1}
                ctrl_opt = 'lqr';
            case{2}
                ctrl_opt = 'pd';
        end
        for pert_mag = [5];            
            n_figcase = n_figcase + 1; figure(n_figcase);            
            PLOT_perf_curves(ctrl_opt, pert_mag, pert_opt, pert_dir, new_save_opt, plot_opt); toc            
        end
    end
    %=====================================================================%
    % FINAL Time-to-Fall Result Files
    pert_opt = 1;
    pert_dir = 'forward';
    for ctrl_opt_sel = 1:2
        switch(ctrl_opt_sel)
            case{1}
                ctrl_opt = 'lqr';
            case{2}
                ctrl_opt = 'pd';
        end
        for pert_mag = [10];            
            n_figcase = n_figcase + 1; figure(n_figcase);
            PLOT_perf_curves(ctrl_opt, pert_mag, pert_opt, pert_dir, new_save_opt, plot_opt); toc            
        end
    end
    %=====================================================================%

end
%=====================================================================================
% PROGRAM DESCRIPTION: Compiling and plotting performance curve data for each controller case
% (CASE: #nodes, controller type: LQR vs PD, perturbation magnitude, perturbation type: growning vs bounded, perturbation direction: forward versus bidrectional) 
% controller from composite controller-characteristic simulation results
% by Raviraj Nataraj, 20150417
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University
function PLOT_perf_curves(ctrl_opt, pert_mag, pert_opt, pert_dir, new_save_opt, plot_opt)

    global QR_sel sim_index n_sim pert_disp
    
    nodes = 50;    
    
    if new_save_opt == 1
        % Compiling controller performance characteristic results from individual simulation files for each controller case (#nodes, closed-loop controller option, perturbation magnitude, perturbation type, perturbation direction)
        ctrl_chars_comp = [];
        for QR_ratio = QR_sel
            for n = sim_index
                load(['walksim results\BE',num2str(nodes),'_',ctrl_opt,'_pert',num2str(pert_mag),'_opt',num2str(pert_opt),'_',pert_dir,'\Results_QR',num2str(QR_ratio),'_n',num2str(n),'.mat']);
                ctrl_chars_comp = [ctrl_chars_comp; ctrl_chars_sim];
            end
        end
    else
        % Load previously saved controller-case results to individual mat-file
        load(['DATA_',ctrl_opt,'_pert',num2str(pert_mag),'_opt',num2str(pert_opt),'_',pert_dir,'.mat']);
    end
    
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

    % Plot raw (all points) performance data versus Q/R
    sx = subplot(2,2,1);
    switch(plot_opt)
        case{1}
            data_var = T_fall; ylabel_name = 'Time-to-fall (Sec)';
        case{2}
            data_var = P_mean3; ylabel_name = 'Control RMS (N-m)';
    end
    px = semilogx(QR_ratio, data_var,'o'); set(px, 'linewidth', 3);
    ax = xlabel('QR-ratio'); bx = ylabel(ylabel_name);
    set(ax, 'fontweight', 'bold'); set(bx, 'fontweight', 'bold'); set(sx, 'fontweight', 'bold');    
    
    % Plot mean+/- performance data versus Q/R
    sx = subplot(2,2,2);
    n_QR = length(QR_ratio)/n_sim;
    for i = 1:n_QR
        p1 = (i-1)*n_sim + 1; p2 = p1 + n_sim - 1;
        QR(i) = mean(QR_ratio(p1:p2));
        m(i) = mean(data_var(p1:p2)); s(i) = std(data_var(p1:p2));
        K(i) = mean(K_mean(p1:p2));
    end        
    px = semilogx(QR, m, 'o', QR, m+s, 'm*', QR, m-s, 'm*'); set(px, 'linewidth', 3);
    ax = xlabel('QR-ratio'); bx = ylabel(ylabel_name);
    set(ax, 'fontweight', 'bold'); set(bx, 'fontweight', 'bold'); set(sx, 'fontweight', 'bold');
    % Superimpose polynomial fit
    N = 2; % polynomial order
    QRexp = log10(QR_sel);
    [P,S] = polyfit(QRexp,m,N); % N-th order polynomial fit data to mean data points
    m_fit = polyval(P,QRexp);
    hold on; px2 = plot(QR, m_fit, 'k-'); set(px2, 'linewidth', 3);
    R = corrcoef(m, m_fit);
    tx = title(['P: ', num2str(P,'%6.2f'), '; R=',num2str(R(2),'%6.2f')]);  
    
    % Plot mean LQR gain versus Q/R
    sx = subplot(2,2,3);
    px = semilogx(QR_ratio, K_mean, '-o'); set(px, 'linewidth', 3);
    ax = xlabel('QR-ratio'); bx = ylabel('mean LQR gain');
    set(ax, 'fontweight', 'bold'); set(bx, 'fontweight', 'bold'); set(sx, 'fontweight', 'bold');
    
    % Plot mean performance data versus mean LQR gain    
    sx = subplot(2,2,4);
    px = plot(K, m, 'o', K, m+s, 'm*', K, m-s, 'm*'); set(px, 'linewidth', 3);
    ax = xlabel('mean LQR gain'); bx = ylabel(ylabel_name);
    set(ax, 'fontweight', 'bold'); set(bx, 'fontweight', 'bold'); set(sx, 'fontweight', 'bold'); 
    % Superimpose polynomial fit
    N = 2; % polynomial order
    [P,S] = polyfit(K,m,N); % N-th order polynomial fit data to mean data points
    m_fit = polyval(P,K);
    hold on; px2 = plot(K, m_fit, 'k-'); set(px2, 'linewidth', 3);
    R = corrcoef(m, m_fit);
    tx = title(['P: ', num2str(P,'%6.2f'), '; R=',num2str(R(2),'%6.2f')]); 
    
    % Legend for controller-case
    legend([ctrl_opt, ', ', pert_dir, ', ',num2str(pert_mag),'N, ' pert_disp{pert_opt}]);

    % Save controller-case results to individual mat-file
    save(['DATA_',ctrl_opt,'_pert',num2str(pert_mag),'_opt',num2str(pert_opt),'_',pert_dir,'.mat'], 'ctrl_chars_comp');
    
end
%=====================================================================================