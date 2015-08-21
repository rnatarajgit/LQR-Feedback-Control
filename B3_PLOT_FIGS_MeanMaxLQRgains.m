% PROGRAM DESCRIPTION: Plotting mean and max LQR gains for as a function
% of Q/R
% by Raviraj Nataraj, 20150810
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

close all; clear all;

% Data (Cut&Paste from 20150810 Manuscript)
DATA = [1e-3	352	17	28	1.7	1439	108	154	17	411	73	81	22;
1e-2	367	21	33	2.2	1461	112	164	22	448	88	127	29;
1e-1	779	26	45	2.7	1782	310	217	32	783	97	202	34;
1	932	37	82	3.6	4132	317	379	50	1023	111	372	39;
2	1112	46	100	4.1	5303	325	438	56	1046	129	474	44;
5	1303	60	124	4.9	6681	377	507	62	1421	159	637	55;
10	1475	72	142	5.7	7472	405	551	66	1960	187	779	66;
20	2079	87	160	6.6	8003	425	589	70	2712	221	932	78;
50	3213	109	186	8.4	8426	439	632	75	4182	272	1138	94;
100	4730	126	208	9.9	8696	445	664	80	5637	313	1289	106;
1000	12199	199	314	17	18319	538	896	98	15812	463	2130	161;
1e4	15851	256	588	24	22426	981	1746	150	20352	634	3190	223];
mean_vals = mean(DATA(:,2:13));

% Assigning data to variables
QR_ratio = DATA(:,1);
gains_joints.Kp_max = DATA(:,2);
gains_joints.Kv_max = DATA(:,3);
gains_joints.Kp_mean = DATA(:,4);
gains_joints.Kv_mean = DATA(:,5);
gains_hip_pos.Kp_max = DATA(:,6);
gains_hip_pos.Kv_max = DATA(:,7);
gains_hip_pos.Kp_mean = DATA(:,8);
gains_hip_pos.Kv_mean = DATA(:,9);
gains_torso.Kp_max = DATA(:,10);
gains_torso.Kv_max = DATA(:,11);
gains_torso.Kp_mean = DATA(:,12);
gains_torso.Kv_mean = DATA(:,13);

% Creating figure plot
figure; x = QR_ratio;
sx = subplot(2,3,1);
y1 = gains_joints.Kp_max; px = semilogx(x,y1); hold on; set(px, 'linewidth', 3);
y2 = gains_joints.Kp_mean; px = semilogx(x,y2); set(px, 'linewidth', 3);
lx = legend('max', 'mean', 'Location', 'NorthWest');
tx = title('JOINTS - K_P');
set(sx, 'fontsize', 12, 'fontweight', 'bold'); 
set(lx, 'fontsize', 10, 'fontweight', 'bold');
set(tx, 'fontsize', 14, 'fontweight', 'bold');
axis([min(x) max(x) 0 max(y1)]);
sx = subplot(2,3,2);
y1 = gains_hip_pos.Kp_max; px = semilogx(x,y1); hold on; set(px, 'linewidth', 3);
y2 = gains_hip_pos.Kp_mean; px = semilogx(x,y2); set(px, 'linewidth', 3);
lx = legend('max', 'mean', 'Location', 'NorthWest');
tx = title('HIP POS - K_P');
set(sx, 'fontsize', 12, 'fontweight', 'bold'); 
set(lx, 'fontsize', 10, 'fontweight', 'bold');
set(tx, 'fontsize', 14, 'fontweight', 'bold');
axis([min(x) max(x) 0 max(y1)]);
sx = subplot(2,3,3);
y1 = gains_torso.Kp_max; px = semilogx(x,y1); hold on; set(px, 'linewidth', 3);
y2 = gains_torso.Kp_mean; px = semilogx(x,y2); set(px, 'linewidth', 3);
lx = legend('max', 'mean', 'Location', 'NorthWest');
tx = title('TORSO - K_P');
set(sx, 'fontsize', 12, 'fontweight', 'bold'); 
set(lx, 'fontsize', 10, 'fontweight', 'bold');
set(tx, 'fontsize', 14, 'fontweight', 'bold');
axis([min(x) max(x) 0 max(y1)]);
sx = subplot(2,3,4);
y1 = gains_joints.Kv_max; px = semilogx(x,y1); hold on; set(px, 'linewidth', 3);
y2 = gains_joints.Kv_mean; px = semilogx(x,y2); set(px, 'linewidth', 3);
lx = legend('max', 'mean', 'Location', 'NorthWest');
tx = title('JOINTS - K_V');
set(sx, 'fontsize', 12, 'fontweight', 'bold'); 
set(lx, 'fontsize', 10, 'fontweight', 'bold');
set(tx, 'fontsize', 14, 'fontweight', 'bold');
axis([min(x) max(x) 0 max(y1)]);
sx = subplot(2,3,5);
y1 = gains_hip_pos.Kv_max; px = semilogx(x,y1); hold on; set(px, 'linewidth', 3);
y2 = gains_hip_pos.Kv_mean; px = semilogx(x,y2); set(px, 'linewidth', 3);
lx = legend('max', 'mean', 'Location', 'NorthWest');
tx = title('HIP POS - K_V');
set(sx, 'fontsize', 12, 'fontweight', 'bold'); 
set(lx, 'fontsize', 10, 'fontweight', 'bold');
set(tx, 'fontsize', 14, 'fontweight', 'bold');
axis([min(x) max(x) 0 max(y1)]);
sx = subplot(2,3,6);
y1 = gains_torso.Kv_max; px = semilogx(x,y1); hold on; set(px, 'linewidth', 3);
y2 = gains_torso.Kv_mean; px = semilogx(x,y2); set(px, 'linewidth', 3);
lx = legend('max', 'mean', 'Location', 'NorthWest');
tx = title('TORSO - K_V');
set(sx, 'fontsize', 12, 'fontweight', 'bold'); 
set(lx, 'fontsize', 10, 'fontweight', 'bold');
set(tx, 'fontsize', 14, 'fontweight', 'bold');
axis([min(x) max(x) 0 max(y1)]);