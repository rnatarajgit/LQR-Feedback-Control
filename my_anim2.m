function my_anim(Result);
% anim.m: make a movie of a simulation result x(t)

    % Loading results file to override input 'Result'
    % load(['walksim results\BE50_lqr_pert5_opt2_bidir\Results_QR1_n1.mat']);
    % load(['walksim results\BE50_lqr_pert10_opt1_forward\Results_QR1_n1.mat']);
    % x = Result.x;
    
    % For half gait-cycle data, creating data with specified #full gait-cycles
    n_cycles = 10;
    sis = [1:3 7:9 4:6 10:12 16:18 13:15];
    xo = Result.x;
    displacement = Result.dur * Result.problem.speed;
    xs = xo(sis,:); xs(1,:) = xs(1,:) + displacement; 
    xo = [xo xs]; 
    x = [];
    for n = 1:n_cycles
        xt = xo; xt(1,:) = xo(1,:) + 2 * displacement * (n - 1);
        x = [x xt];
    end   
    
	nframes = size(x,2);	

	fps = 25;
	
	% initialize movie file
	avi = VideoWriter('anim.avi');
	open(avi);

	% initialize figure window
	close all
	figure(1);
	clf;
	set(gcf,'Position',[10 10 550 550]);
	set(gcf, 'color', 'white');
	
	% determine size of ground
	xmin = min(x(1,:)) - 0.5;
	xmax = max(x(1,:)) + 0.5;
	
	% determine how much the model moves per frame
	speed = (max(x(1,:)) - min(x(1,:))) / nframes;
	
	% create ground points (random dots)
	np = 5000*round(xmax-xmin);
	xg = rand(np,1);
	yg = rand(np,1);
	xg = xmin + (xmax-xmin)*[xg ; 2-xg];
	yg = -0.15*[yg ; yg];
	
	% make the movie
	R = [1:6 4];			% right stick points
	L = [2 7:10 8];			% left stick points
	u = zeros(9,1);
	for i=0:nframes-1
		plot(xg,yg,'.','Color',[0.7 0.7 0.7],'MarkerSize',4);
		hold on
		[xdot, grf, d] = gait2dem(x(:,i+1),u);
		d = reshape(d',2,10)';
		plot(d(R,1),d(R,2),'r',d(L,1),d(L,2),'b','LineWidth',2);
		axis('equal');
		axis([-1+speed*i 1+speed*i -0.2 1.5]);
		axis('off');
		if (i==0)
			F = getframe(gca);
			frame = [1 1 size(F.cdata,2) size(F.cdata,1)];
		else
			F = getframe(gca,frame);
		end
		writeVideo(avi,F);
		cla;
	end

	close(avi);
	hold off;
	close all
end