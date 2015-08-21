function opt
	% run a sequence of optimizations
	clear all
    
	% general settings
	problem.Solver = 'IPOPT';
	problem.MaxIterations = 2000;
	problem.ConstraintTol = .0001;
	problem.Tol = .0001;
	problem.symmetry = 1;
	problem.discretization = 'BE';		% start with Backward Euler, this converges more easily
	problem.checkderivatives = 0;		% set this to 1 to make optim.m check derivatives
	problem.N = 20;						% start with a coarse mesh    
    
	% settings for this optimization
	problem.gaitdata = 'Winter_normal.mat';
	load(problem.gaitdata);
	problem.dur = gait.dur(1)/2;		% prescribed duration of half gait cycle
    problem.Weffort = 1;				% weighting for the squared-torque cost
	problem.Wreg = 0.000001;			% regularization term
	
	% do the first optimization to find a minimal-effort standing solution
	% initial guess for all unknowns is the midpoint between upper and lower bound
	problem.speed = 0.0;
	problem.Wtrack = 1e-8;
	do_opt(problem, 'mid', 'result001.mat');
	
	% now introduce some tracking, the model will start doing small steps without moving forward
	problem.Wtrack = 0.01;
	do_opt(problem, 'result001.mat', 'result002.mat');

	% a slow walk and more tracking
	problem.Wtrack = 0.1;
	problem.speed = 0.5;
	do_opt(problem, 'result002.mat', 'result003.mat');
	
	% the final tracking at the right speed
	problem.N = 50;    
    problem.Wtrack = 0.1;
	problem.speed = 1.3250;
	do_opt(problem, 'result003.mat', 'result004.mat');    

	% and now with 400 nodes
	problem.N = 400;    
	do_opt(problem, 'result004.mat', 'result005.mat');
 	
 	% and now with Midpoint Euler discretization
 	problem.discretization = 'ME';
 	do_opt(problem, 'result005.mat', 'result006.mat');

	% creating custom trajectory solutions with various #nodes and initial guesses
	problem.N = 350;
    problem.discretization = 'BE';
    do_opt(problem, 'result006_ME400.mat', 'result006_BE350.mat');
	
end
%====================================================================
function problem = do_opt(problem, initialguess, resultfile)
	problem.initialguess = initialguess;
	problem.resultfile 	 = resultfile;
	if (exist(resultfile)==2)
		fprintf('Result file %s already exists, nothing to do.\n', resultfile);
	else
		fprintf('Working on %s...\n', resultfile);
		optim(problem);
		fprintf('Optimization has been completed.  Result is in %s.\n', resultfile);
		disp('Press ENTER to continue');
		pause
	end
end



