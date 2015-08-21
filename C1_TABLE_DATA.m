% PROGRAM DESCRIPTION: Tabulating/outputting table data for LQR 
% gait-control paper (inc. RMS for state errors, torques)
% by Raviraj Nataraj, 20150702
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

function C1_TABLE_DATA

tic;

% TABULATING DATA FOR TABLE
fdbk_name{1} = 'pd'; fdbk_name{2} = 'lqr'; n_fdbk = length(fdbk_name);
n_sim = 20;
QR = [1e-3 1e-2 1e-1 1e0 2e0 5e0 1e1 2e1 5e1 1e2 1e3 1e4];
n_ctrl = length(QR);
for fdbk = 1:2
    for ctrl = 1:n_ctrl
        for sim = 1:n_sim
            [fdbk QR(ctrl) sim]
            fname = ['walksim results\BE50_',fdbk_name{fdbk},'_pert5_opt2_bidir\Results_QR',num2str(QR(ctrl)),'_n',num2str(sim),'.mat'];
            load(fname);
            res = Result_walksim; clear Result_walksim;
            err = res.X - res.X_des;
            U_cloop = res.U_app - res.U_open;
            k = res.K;
            n_jt = size(k,2);
            for jt = 1:n_jt
                k_jt = squeeze(k(:,jt,:));
                toq{jt} = -k_jt.*err;
            end
            rms_err(sim,:) = rms(err);
            for jt = 1:n_jt
                rms_toq{jt}(sim,:) = rms(toq{jt});
            end
        end
        RMS_ERR_mean(fdbk, ctrl, :) = mean(rms_err);
        RMS_ERR_std(fdbk, ctrl, :) = std(rms_err);
        for jt = 1:n_jt
            RMS_TOQ_mean{jt}(fdbk, ctrl, :) = mean(rms_toq{jt});
            RMS_TOQ_std{jt}(fdbk, ctrl, :) = std(rms_toq{jt});
        end
        GAIN{ctrl}.ctrl = QR(ctrl);
        GAIN{ctrl}.mean = squeeze(mean(abs(k)));
        GAIN{ctrl}.max = squeeze(max(abs(k)));
        GAIN{ctrl}.std = squeeze(std(abs(k)));
    end
end

% PRINTING INFORMATION FOR EACH CONTROLLER ON SEPARATE SHEET
fdbk_name{1} = 'PD'; fdbk_name{2} = 'LQR'; n_fdbk = length(fdbk_name);
QR = [1e-3 1e-2 1e-1 1e0 2e0 5e0 1e1 2e1 5e1 1e2 1e3 1e4];
n_ctrl = length(QR);
st_name = { 'HIP-Xpos','HIP-Ypos','TORSO-TILT',...
    'RHIP-ang','RKNE-ang','RANK-ang',...
    'LHIP-ang','LKNE-ang','LANK-ang',...
    'HIP-Xpos-deriv','HIP-Ypos-deriv','TORSO-TILT-deriv',...
    'RHIP-ang-deriv','RKNE-ang-deriv','RANK-ang-deriv',...
    'LHIP-ang-deriv','LKNE-ang-deriv','LANK-ang-deriv','SCALING FACTOR'};
jt_name_toq = {'RHIP-toq', 'RKNE-toq', 'RANK-toq', 'LHIP-toq', 'LKNE-toq', 'LANK-toq'};
jt_name_gain_max = {'RHIP-gain_max', 'RKNE-gain_max', 'RANK-gain_max', 'LHIP-gain_max', 'LKNE-gain_max', 'LANK-gain_max'};
jt_name_gain_mean = {'RHIP-gain_mean', 'RKNE-gain_mean', 'RANK-gain_mean', 'LHIP-gain_mean', 'LKNE-gain_mean', 'LANK-gain_mean'};
RMS_ERR_mean_tot = zeros(18,1);
RMS_TOQ_mean_tot = zeros(6,18);
GAIN_max_tot = zeros(6,18);
GAIN_mean_tot = zeros(6,18);
for fdbk = 2
    % OUTPUTTING RESULTS FOR INDIVIDUAL CONTROLLERS
    for ctrl = 1:n_ctrl
        sheet_name = [fdbk_name{fdbk},', QR_ratio=',num2str(QR(ctrl))]
        toc
        xlswrite('TABLE_DATA.xls',st_name',sheet_name,'A2'); xlswrite('TABLE_DATA.xls',st_name(1:18)',sheet_name,'J2'); xlswrite('TABLE_DATA.xls',st_name(1:18)',sheet_name,'R2');
        xlswrite('TABLE_DATA.xls',['RMS-error', jt_name_toq], sheet_name,'B1'); xlswrite('TABLE_DATA.xls',jt_name_gain_max, sheet_name,'K1'); xlswrite('TABLE_DATA.xls', jt_name_gain_mean, sheet_name,'S1');
        jt_col = {'B','C','D','E','F','G','H','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','AA'};
        xlswrite('TABLE_DATA.xls',squeeze(RMS_ERR_mean(fdbk,ctrl,:)), sheet_name,[jt_col{1},'2']);
        for jt = 1:6
            sf = sum(squeeze(RMS_TOQ_mean{jt}(fdbk,ctrl,:)));
            xlswrite('TABLE_DATA.xls',[squeeze(RMS_TOQ_mean{jt}(fdbk,ctrl,:))/sf*100; sf/100], sheet_name,[jt_col{jt+1},'2']);
            xlswrite('TABLE_DATA.xls',squeeze(GAIN{ctrl}.max(jt,:))', sheet_name,[jt_col{jt+7},'2']);
            xlswrite('TABLE_DATA.xls',squeeze(GAIN{ctrl}.mean(jt,:)'), sheet_name,[jt_col{jt+15},'2']);
            
        end
        % Tabulating results across all controllers
        RMS_ERR_mean_tot = RMS_ERR_mean_tot + squeeze(RMS_ERR_mean(fdbk,ctrl,:))/n_ctrl;
        for jt = 1:6
            RMS_TOQ_mean_tot(jt,:) = RMS_TOQ_mean_tot(jt,:) + squeeze(RMS_TOQ_mean{jt}(fdbk,ctrl,:))'/n_ctrl;
            GAIN_max_tot(jt,:) = GAIN_max_tot(jt,:) + squeeze(GAIN{ctrl}.max(jt,:))/n_ctrl;
            GAIN_mean_tot(jt,:) = GAIN_mean_tot(jt,:) + squeeze(GAIN{ctrl}.mean(jt,:))/n_ctrl;
        end
    end
    
    % OUTPUTTING TOTAL RESULTS ACROSS ALL CONTROLLERS
    sheet_name = [fdbk_name{fdbk},', TOTAL-All Controllers']
    toc
    xlswrite('TABLE_DATA.xls',st_name',sheet_name,'A2'); xlswrite('TABLE_DATA.xls',st_name(1:18)',sheet_name,'J2'); xlswrite('TABLE_DATA.xls',st_name(1:18)',sheet_name,'R2');
    xlswrite('TABLE_DATA.xls',['RMS-error', jt_name_toq], sheet_name,'B1'); xlswrite('TABLE_DATA.xls',jt_name_gain_max, sheet_name,'K1'); xlswrite('TABLE_DATA.xls', jt_name_gain_mean, sheet_name,'S1');
    jt_col = {'B','C','D','E','F','G','H','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','AA'};
    xlswrite('TABLE_DATA.xls',squeeze(RMS_ERR_mean(fdbk,ctrl,:)), sheet_name,[jt_col{1},'2']);
    for jt = 1:6
        sf = sum(RMS_TOQ_mean_tot(jt,:));
        xlswrite('TABLE_DATA.xls',[RMS_TOQ_mean_tot(jt,:)'/sf*100; sf/100], sheet_name,[jt_col{jt+1},'2']);
        xlswrite('TABLE_DATA.xls',GAIN_max_tot(jt,:)',sheet_name,[jt_col{jt+7},'2']);
        xlswrite('TABLE_DATA.xls',GAIN_mean_tot(jt,:)',sheet_name,[jt_col{jt+15},'2']);
    end
    
end

toc

end