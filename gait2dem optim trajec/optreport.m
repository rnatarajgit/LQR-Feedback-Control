function report(resultfile, initialize)

% initialize figure window
close all
figure(1);
clf;
set(gcf,'Position',[5 5 815 960]);

% Loading the Result structure form the mat-file specified
load(resultfile);
x = Result.x;
u = Result.u;
dur = Result.dur;
speed = Result.problem.speed;
vmx = Result.problem.vmx;
vmu = Result.problem.vmu;
symm = Result.problem.symmetry;
N = size(x,2);

% Vector of time instants for nodes
T = (1:N)*dur/N; 

% Computing GRFs and muscle forces
[xdot, GRF, stick, mom] = gait2dem(x,u);

% construct a full simulated gait cycle from a half cycle, or put left cycle after right cycle
simhip = 180/pi*[x(4,:) x(7,:)]'; 
simknee = 180/pi*[x(5,:) x(8,:)]';
simankle = 180/pi*[x(6,:) x(9,:)]';
simgrfx = [GRF(1,:) GRF(4,:)]';
simgrfy = [GRF(2,:) GRF(5,:)]';
simmom = [mom(1:3,:) mom(4:6,:)]';	

% same for gait data
data   = Result.problem.trackdata.av;
datasd = Result.problem.trackdata.sd;
datahip =   180/pi*[data(:,1) ; data(:,6)];
dataknee =  180/pi*[data(:,2) ; data(:,7)];
dataankle = 180/pi*[data(:,3) ; data(:,8)];
datagrfx =  [data(:,4) ; data(:,9)];
datagrfy =  [data(:,5) ; data(:,10)];
sdhip =   180/pi*[datasd(:,1) ; datasd(:,6)];
sdknee =  180/pi*[datasd(:,2) ; datasd(:,7)];
sdankle = 180/pi*[datasd(:,3) ; datasd(:,8)];
sdgrfx =  [datasd(:,4) ; datasd(:,9)];
sdgrfy =  [datasd(:,5) ; datasd(:,10)];

% determine BW from mean vertical GRF, and normalize
BW = 2*mean(mean(simgrfy));
simgrfx = simgrfx/BW;
simgrfy = simgrfy/BW;
datagrfx = datagrfx/BW;
datagrfy = datagrfy/BW;
sdgrfx = sdgrfx/BW;
sdgrfy = sdgrfy/BW;

% Plot joint angles
subplot(3,3,1)
plotvar(simhip, 'Hip Angle', symm, datahip, sdhip);
ylabel('[degrees]','FontSize',10)
set(gca,'XTickLabel',{});
subplot(3,3,4)
plotvar(-simknee, 'Knee Angle', symm, -dataknee, sdknee);
set(gca,'XTickLabel',{});
subplot(3,3,7)
plotvar(simankle, 'Ankle Angle', symm, dataankle, sdankle);
set(gca,'XTickLabel',{});
if (symm == 0)
	legend('Right','Left');
end

% Plot joint moments
subplot(3,3,2)
plotvar(simmom(:,1), 'Hip Moment', symm);
ylabel('[degrees]','FontSize',10)
set(gca,'XTickLabel',{});
subplot(3,3,5)
plotvar(simmom(:,2), 'Knee Moment', symm);
set(gca,'XTickLabel',{});
subplot(3,3,8)
plotvar(simmom(:,3), 'Ankle Moment', symm);
set(gca,'XTickLabel',{});
if (symm == 0)
	legend('Right','Left');
end

% Optimization Information
subplot(3,3,3)
text(0,0.8,['# Nodes: ' num2str(N)],'FontSize',10)
text(0,0.7,['File name: ' strrep(resultfile,'_','\_')],'FontSize',10)
text(0,0.6,['Speed: ' num2str(speed,'%8.3f') ' m/s'],'FontSize',10)
if (Result.problem.symmetry)
	factor = 2;
else
	factor = 1;
end
text(0,0.50,['Gait cycle: ', num2str(factor*Result.dur,'%8.3f'),' s'],'FontSize',10);
axis off
box on

% Vertical Ground reaction force
subplot(3,3,6)
plotvar(simgrfy, 'GRF Y', symm, datagrfy, sdgrfy);
set(gca,'XTickLabel',{});
ylabel('[BW]');

% Horizontal GRF
subplot(3,3,9)
plotvar(simgrfx, 'GRF X', symm, datagrfx, sdgrfx);
ylabel('[BW]');
xlabel('Time [% of gait cycle]');

end
%============================================================================
function plotvar(sim,name,bsymm,av,sd);
	hold on	
	% do we have symmetry or do we plot left and right separately?
	if (bsymm)
		N = size(sim,1);
		t = 100*(0:N)/N;
		if (nargin>3)
			x = [t  t(end:-1:1)];
			Rav = [av(1:N); av(1)];
			Rsd = [sd(1:N); sd(1)];		
			y = [Rav-Rsd ; Rav(end:-1:1)+Rsd(end:-1:1)];	
			fill(x,y,[0.8 0.8 1], 'FaceAlpha',0.5);
		end
		plot(t, [sim; sim(1)],'b','LineWidth',1.5);
	else
		N = size(sim,1)/2;
		t = 100*(0:N)/N;
		Rsim = [sim(1:N); sim(1)];
		Lsim = [sim(N+1:2*N); sim(N+1)];
		if (nargin>3)
			x = [t  t(end:-1:1)];
			Rav = [av(1:N); av(1)];
			Lav = [av(N+1:2*N); av(N+1)];
			Rsd = [sd(1:N); sd(1)];
			Lsd = [sd(N+1:2*N); sd(N+1)];			
			Ry = [Rav-Rsd ; Rav(end:-1:1)+Rsd(end:-1:1)];	
			Ly = [Lav-Lsd ; Lav(end:-1:1)+Lsd(end:-1:1)];	
			fill(x,Ry,[0.8 0.8 1], 'FaceAlpha',0.5);
			fill(x,Ly,[1 0.8 0.8], 'FaceAlpha',0.5);
		end
		plot(t, Rsim,'b','LineWidth',1.5);
		plot(t, Lsim,'r','LineWidth',1.5);
	end
	box on
	hold off
	if (nargin>3)
		ymin = min([av-sd;sim]);
		ymax = max([av+sd;sim]);
	else
		ymin = min(sim);
		ymax = max(sim);
	end
	margin = 0.05*(ymax-ymin);
	ymin = ymin - margin;
	ymax = ymax + margin;
	axis([0 100 ymin ymax]);
	if (~isempty(name))
		text(98,ymax-margin/2,name,'HorizontalAlignment','Right','VerticalAlignment','Top','FontSize',10);
	end
	set(gca,'Fontsize',10);
	box on
end
