function [result] = optim(input_problem)
	% trajectory optimization for gait2dem model
	tic;
	clear global
	global problem
	problem = input_problem;
    
	initialguess = problem.initialguess;

	% Make sure that model is re-initialized
	clear mex
	
	% store mass of the model
	mass = 75.0;
	gravity = 9.81;
	problem.weight = mass*gravity;
	
	% settings from "problem" struct
	N			= problem.N;				% number of collocation nodes
	if mod(N,2)~=0
		error('N must be an even number.');
	end

	% some constants for the direct collocation problem
	nstates = 18;
	ncontrols = 9;
	nvarpernode = nstates + ncontrols;
	nvar = N*nvarpernode + 1;				% N state/control samples, and duration
	ncon = N*nstates;
	
	% construct mirroring vectors, for periodicity constraint
	if (problem.symmetry)
		problem.vmx = [1:3 7:9 4:6 10:12 16:18 13:15];
		problem.vmu = [1:3 7:9 4:6];
	else
		problem.vmx = (1:18);
		problem.vmu = (1:9);
	end
    
	% store some things in problem struct
	problem.nstates = nstates;
	problem.ncontrols = ncontrols;
	problem.nvarpernode = nvarpernode;
	problem.nvar = nvar;
	problem.ncon = ncon;
	
	%--------------------------------------------------------------------
	% load gait data, if needed
	if problem.Wtrack ~= 0
		if ~isfield(problem, 'gaitdata')
			error('gait data filename was missing');
		end
		problem = loaddata(problem);
	end

	% initialize data structure for progress logging
	problem.optlog.conval = 0;
	problem.optlog.data = [];
	problem.optlog.print = 0;
	
	%------------------------------------------------------------------------
	% Lower and upper bounds for the optimization variables X
	% X will contain, for each node, the states and controls.
	%---------------------------------------------------------------------------------
	minq = [-3 0.5 -pi/3 -pi -pi -pi -pi -pi -pi]';			% bounds for position variables q
	maxq = [ 3 1.5  pi/3  pi  pi  pi  pi  pi  pi]';
	minqdot = [-10 -10 -20 -20 -20 -20 -20 -20 -20]';		% bounds for velocity variables qdot
	maxqdot = [ 10  10  20  20  20  20  20  20  20]';
	minu = [0 0 0 -300 -300 -300 -300 -300 -300]';			% bounds for controls
	maxu = [0 0 0  300  300  300  300  300  300]';
	L = zeros(nvar,1);
	U = zeros(nvar,1);
	for k = 0:N-1
		L(k*nvarpernode + (1:nvarpernode) ) = [minq; minqdot; minu]; 
		U(k*nvarpernode + (1:nvarpernode) ) = [maxq; maxqdot; maxu];
	end
	problem.L = L;
	problem.U = U;
	
	% movement duration (may be prescribed in problem)
	if isfield(problem,'dur') && ~isempty(problem.dur)
		L(end) = problem.dur;
		U(end) = problem.dur;
		fprintf('Movement duration prescribed:    %9.4f s\n',problem.dur);  
	else
		L(end) = 0.2;
		U(end) = 2.0;		% duration between 0.2 and 2.0 seconds
	end	

	% constrain X of trunk at first node to be zero (since it does not matter, and this helps convergence)
	L(1) = 0;
	U(1) = 0;
	
	% also store the midpoint between bounds
	problem.M = (L+U)/2;
	
	% scaling factors for effort in cost function, joint moments need to be divided by 100
	problem.umax = [1 1 1 100 100 100 100 100 100]';
	
	%------------------------------------------------------------------------------
	% make an initial guess
	if ~isempty(strfind(initialguess, 'mid'))
		X0 = problem.M;								% halfway between upper and lower bound 
	elseif ~isempty(strfind(initialguess, 'random'))
		X0 = L + (U - L).*rand(size(L));			% random between upper and lower bound 
	else
		% load a previous solution, filename is before first comma
		i = strfind(initialguess,',');
		if isempty(i)
			i = size(initialguess,2);
		else
			i = i-1;
		end
		initialguessfile = initialguess(1:i);
		load(initialguessfile);
		Nresult = size(Result.x,2);
		t0 = (0:(Nresult-1))'/Nresult;
		x0 = Result.x';
		u0 = Result.u';
		
		% if we are going from asymmetrical to symmetrical solution, give error message
		if problem.symmetry && ~Result.problem.symmetry
			error('optim: cannot use asymmetrical initial guess for symmetrical problem');
		end
		
		% if we are going from a symmetrical to an asymmetrical solution, create a full gait cycle now
		dur = Result.dur;
		if ~problem.symmetry && Result.problem.symmetry
			dur = 2*dur;
			t0 = 0.5*[t0 ; t0+1];
			x0secondhalf = x0(:,Result.problem.vmx);
			u0secondhalf = u0(:,Result.problem.vmu);
			x0secondhalf(:,1) = x0(:,1) + problem.speed * Result.dur;
			x0 = [x0 ; x0secondhalf];
			u0 = [u0 ; u0secondhalf];
		end
		
		% add one node so we can interpolate with periodicity
		t0 = [t0 ; 1];
		u0 = [u0 ; u0(1,problem.vmu)];
		x0 = [x0 ; x0(1,problem.vmx)];
		x0(end,1) = x0(end,1) + problem.speed * Result.dur;
		
		% interpolate states and controls from initial guess to current node times
		times = (0:(N-1))/N;
		x0 = interp1(t0,x0,times,'linear','extrap');
		u0 = interp1(t0,u0,times,'linear','extrap');
		X0 = reshape([x0 u0]',nvar-1,1);
		X0 = [X0 ; dur];
		

	end
	X0 = X0 + 0.001*randn(size(X0));		% add some randomness
	problem.X0 = X0;
	
	% finally if ',adjustgrf' is specified as part of initial guess, adjust vertical position to make mean(GRF) = body weight
	if ~isempty(strfind(initialguess,',adjustgrf'))
		done = 0;
		a1 = -5;
		a2 = +5;
		a = 0.0;
		g = 9.81;
		X = X0;
		iter = 0;
		while (~done)
			iter = iter+1;
			if (iter > 50)
				error('Could not complete adjustgrf.  Please restart with different initial guess, or remove adjustgrf option.');
			end
			% shift vertical position by amount a
			X(2:nvarpernode:nvar-1) = X0(2:nvarpernode:nvar-1) + a;
			% evaluate vertical GRF at each node, and sum them
			ix = 1:problem.nstates;			% index to state of first node
			sum = 0;
			for i=1:N
				x = X(ix);
				[~,GRF] = gait2dem(x,zeros(ncontrols,1));
				sum = sum + GRF(2) + GRF(4);
				ix = ix + nvarpernode;						% move pointers to next node
			end
			meangrf = sum/N/mass;
			fprintf('Mean VGRF/m: %8.3f  at shift of %8.4f m\n',meangrf,a);
			if (meangrf > g + 0.001)		% shift model up
				a1 = a;
				a = (a + a2)/2;
			elseif (meangrf < g - 0.001)	% shift model down
				a2 = a;
				a = (a + a1)/2;
			else
				done = 1;
			end
		end
		X0 = X;
	end

	%---------------------------------------------------------------------------------
	% determine sparsity structure of Jacobian, the slow but safe way
	% I have verified that we always get same structure by testing with random X
	disp('Determining sparsity structure of constraint Jacobian...');
	
	% initial estimate of pattern is based on repeated blocks of nstates x nvarpernode
	% this initial estimate is needed for conjac to work
	problem.Jpattern = spalloc(ncon,nvar,2*N*nstates*nvarpernode + ncon);
	
	% last column is full
	problem.Jpattern(:,end) = 1;
	
	% now the blocks due to the finite differencing
	rows = 1:nstates;
	cols = 1:2*nvarpernode;
	for i = 1:N
		if (i==N)					% on last node, periodicity constraint fills the first nvarpernode columns
			cols(nvarpernode+1:end) = 1:nvarpernode;
		end
		problem.Jpattern(rows,cols) = 1;
		rows = rows + nstates;
		cols = cols + nvarpernode;
		if (toc > 5)
			fprintf('Node %d of %d\n', i, N);
			tic;
		end
	end
	Jpattern1 = problem.Jpattern;
	problem.Jnnz = nnz(problem.Jpattern);
	fprintf('Jacobian sparsity: %d nonzero elements out of %d (%5.3f%%).\n',problem.Jnnz, ncon*nvar, 100*problem.Jnnz/(ncon*nvar));
	
	% now refine it with an actual Jacobian evaluation, this will reveal more zeros
	% choose a random X but make sure the horizontal velocity is not too high (the logistic friction model saturates quickly and nonzeros can be missed!)
	X = (L+U)/2 + 0.001*rand(size(L)).*(U-L);
	J = conjac(X);
	problem.Jnnz = nnz(J);
	fprintf('Jacobian sparsity: %d nonzero elements out of %d (%5.3f%%).\n',problem.Jnnz, ncon*nvar, 100*problem.Jnnz/(ncon*nvar));
	problem.Jpattern = double(J~=0);
	%----------------------------------------------------------------------------------
	% check the derivatives, compare between slow and fast calculation method
	if isfield(problem,'checkderivatives') && problem.checkderivatives
		problem.print = 0;
		hh = 1e-6;
		X = L + (U-L).*rand(size(L));		% a random vector of unknowns within upper and lower bounds
		f = objfun(X);
		grad = objgrad(X);
		c = confun(X);
		cjac = conjac(X);
		cjac_num = zeros(ncon,nvar);
		grad_num = zeros(nvar,1);
		tic;
		for i=1:nvar
			if (toc > 2.0)
				fprintf('checking objgrad and conjac for unknown %4d of %4d\n',i,nvar);
				tic;
			end
			Xisave = X(i);
			X(i) = X(i) + hh;
			cjac_num(:,i) = (confun(X) - c)/hh;
			grad_num(i) =   (objfun(X) - f)/hh;
			X(i) = Xisave;
		end
		
		% check the Jacobian pattern
		patdiff = problem.Jpattern - (cjac~=0);
		if (min(patdiff) < 0)
			disp('cjac has nonzeros that Jpattern has missed.');
			keyboard
		end
		
		% report maximal differences between the two results
		fprintf('Max. error in constraint jacobian: ');
		matcompare(cjac, cjac_num);
		fprintf('Max. error in objective gradient: ');
		matcompare(grad, grad_num);
		keyboard
	end
	
	%------------------------------------------------------------------------------------
	screen = get(0,'ScreenSize');
	close all;
	figure(1);		% all plotting during optimization is done in this figure
	clf;
	set(gcf,'Position',[5 screen(4)-390 700 320]);

	%------------------------------------------------------------------------------------
	% evaluate initial guess
	fprintf('Initial guess evaluation:\n');
	problem.print = 1; 
	confun(X0);
	objfun(X0);

	%----------------------------------------------------------------------------------------
	% do the optimization
	if (problem.MaxIterations > 0)
		fprintf('Starting optimization...\n');
		problem.print = 0;
		funcs.objective = @objfun;
		funcs.gradient  = @objgrad;
		funcs.constraints = @confun;
		funcs.jacobian    = @conjac;
		funcs.jacobianstructure = @conjacstructure;
		options.lb = L;
		options.ub = U;
		options.cl = zeros(ncon,1);
		options.cu = zeros(ncon,1);	
		options.ipopt.max_iter = problem.MaxIterations;
		options.ipopt.hessian_approximation = 'limited-memory';
		options.ipopt.tol = problem.Tol;
		options.ipopt.constr_viol_tol = problem.ConstraintTol;
		options.ipopt.compl_inf_tol = problem.ConstraintTol;
		options.ipopt.linear_solver = 'mumps';        % default is ma27, options: ma57, mumps (requires library install)
        options.ipopt.print_level = 1;
		[X, info] = ipopt(X0,funcs,options);
		disp(['IPOPT status: ' ipoptmessage(info.status)]);
		result.info = info.status;
	else		% skip optimization
		X = X0;
		result.info = 0;		% indicate success because optimization was not requested
	end
	fprintf('Duration of movement: %8.4f s\n',X(end));
    
    % save optimization result on file   
    problem.J = conjac(X); 
	savefile(X, problem.resultfile, problem); 
	disp(['Optimization result was saved in the file ' problem.resultfile]);
	
	% show stick figure of result and optimization log
	disp('Result of optimization:');
	problem.print = 1;
	objfun(X);
	confun(X);
	
end		% end of function optim
%===========================================================================================
function f = objfun(X)
	% calculates the objective function that is minimized
	global problem
	
	N = problem.N;
	Nstates = problem.nstates;
	Ncontrols = problem.ncontrols;
	Nvarpernode = problem.nvarpernode;

	% Extract the states and controls from the vector of optimization variables X
	dur = X(end);
	xu = reshape(X(1:end-1),Nvarpernode,N);
	xx = xu(1:Nstates,:);
	uu = xu(Nstates+1:Nvarpernode,:);
	
	% tracking term
	if (problem.Wtrack ~= 0)
		% Use model to calculate the GRF in all nodes with one function call
		[~, GRF] = gait2dem(xx,uu);
		simdata = [xx(4:6,:)' GRF(1:2,:)' xx(7:9,:)' GRF(4:5,:)'];						% simulated angles and grf in all nodes
		residuals = (problem.trackdata.av - simdata)./problem.trackdata.sd;
		mresiduals = mean(residuals.^2);			% average the squared residuals over time
		mresiduals = [mresiduals , ((problem.trackdata.dur - dur)/problem.trackdata.dursd)^2];	% add residual for subject gait cycle duration
		f1 = problem.Wtrack*mean(mresiduals);			% now average over all variables
	else
		f1 = 0;
	end
	
	% effort term
	f2 = 0;
	if (problem.Weffort ~= 0)
		for k=1:N
			iP = (k-1)*Nvarpernode + Nstates + (1:Ncontrols);
			f2 = f2 + problem.Weffort * sum(X(iP).^2 ./ problem.umax.^2) / N / Ncontrols;
		end		
	end
	
	% regularization term
	f3 = 0;
	if (problem.Wreg ~= 0)
		% create an extra node via periodicity condition so we can do derivatives
		xu_last = xu([problem.vmx Nstates+problem.vmu],1);
		xu_last(1) = xu_last(1) + problem.speed * X(end);
		xud = diff([xu xu_last]');
		f3 = problem.Wreg * N * mean(sum(xud.^2));
	end
		
	f = f1 + f2 + f3;

	problem.optlog.data = [problem.optlog.data ; problem.optlog.conval f f1 f2 f3];

	% print something on screen every 3 seconds, or when specifically requested
	if (toc > 3.0 || problem.print)
		fprintf('%d -- Normc: %8.6f  ', size(problem.optlog.data,1), problem.optlog.conval);
		fprintf('Obj: %8.5f = %8.5f (track) + %8.5f (effort) + %8.5f (reg)\n', f, f1, f2, f3);
		report(X);
		tic;
	end
	
end
%===========================================================================================
function [grad] = objgrad(X)
	% calculates the gradient of the objective function
	global problem
	
	N = problem.N;
	Nvarpernode = problem.nvarpernode;
	Nstates = problem.nstates;
	Ncontrols = problem.ncontrols;
	
	% Extract the states and controls from the vector of optimization variables X
	dur = X(end);
	xnodes = reshape(X(1:end-1),Nvarpernode,N);
	xx = xnodes(1:Nstates,:);
	uu = xnodes(Nstates+1:Nvarpernode,:);
	
	grad = zeros(size(X));
	
	if (problem.Wtrack ~= 0)
		grfav = problem.trackdata.av(:,[4 5 9 10])';
		grfsd = problem.trackdata.sd(:,[4 5 9 10])';
		angav = problem.trackdata.av(:,[1 2 3 6 7 8])';
		angsd = problem.trackdata.sd(:,[1 2 3 6 7 8])';

		% Use model to calculate the GRF in all nodes with one function call
		[~, GRF] = gait2dem(xx,uu);
		GRF = GRF([1 2 4 5],:);				% only use Fx,Fy on each foot

		hh = 1e-7;
		% GRF only depends only on 17 kinematic variables (x2..x18) in each node, and nodes are not coupled
		% so we can calculate derivatives simultaneously
		dGRFdX = zeros(4, N, 17);
		for i=1:17
			xxisave = xx(1+i,:);
			xx(1+i,:) = xxisave + hh;
			[~, GRFhh] = gait2dem(xx,uu);
			GRFhh = GRFhh([1 2 4 5],:);				% only use Fx,Fy on each foot
			dGRFdX(:,:,i) = (GRFhh-GRF)/hh;
			xx(1+i,:) = xxisave;
		end

		for k=1:N
			iP = (k-1)*Nvarpernode + (2:18);
			grad(iP) = grad(iP) + 2 * problem.Wtrack * squeeze(dGRFdX(:,k,:))' * ( (GRF(:,k) - grfav(:,k))./grfsd(:,k).^2 ) / N / 11;
			
			iP = (k-1)*Nvarpernode + (4:9);		% pointer to the angles in X
			grad(iP) = grad(iP) + 2 * problem.Wtrack * ( (X(iP) - angav(:,k))./angsd(:,k).^2) / N / 11;
		end

		% duration term in cost function
		grad(end) = grad(end) + 2 * problem.Wtrack*(X(end) - problem.trackdata.dur)/problem.trackdata.dursd^2 / 11;	

	end	
	
	if (problem.Weffort ~= 0)
		for k=1:N
			iP = (k-1)*Nvarpernode + Nstates + (1:Ncontrols);
			grad(iP) = grad(iP) + 2 * problem.Weffort * X(iP) ./ problem.umax.^2 / N / Ncontrols;
		end		
	end

	% regularization term
	if (problem.Wreg ~= 0)
		ix = 1:Nvarpernode;			% index to states and controls of first node
		factor = problem.Wreg*N/Nvarpernode;
		for i=1:N
			x1 = X(ix);
			if (i<N)
				x2 = X(ix+Nvarpernode);
			else
				vm = [problem.vmx Nstates+problem.vmu];
				x2 = X(vm);
				x2(1) = x2(1) + problem.speed * X(end);
			end
			xd = x2 - x1;
			
			% now construct the gradient contributions of this term which is factor*sum(xd.^2)
			grad(ix) = grad(ix) - 2*factor * xd;			
			if (i<N)
				grad(ix+Nvarpernode) = grad(ix+Nvarpernode) + 2*factor*xd;
			else
				grad(vm) = grad(vm) + 2*factor*xd;
				grad(end) = grad(end) + 2 * factor * problem.speed * xd(1);
			end
			ix = ix + Nvarpernode;	% move pointers to next node
		end	

	end	
		
end
%===========================================================================================
function c = confun(X)
	% returns the constraint vector
	global problem
	
	nstates 		= problem.nstates;
	ncontrols       = problem.ncontrols;
	nvarpernode 	= problem.nvarpernode;
	ncon 			= problem.ncon;
	vmx 			= problem.vmx;
	vmu 			= problem.vmu;
	N 				= problem.N;
	
	h 	= X(end)/N;			% time interval
	
	% extract states and controls from X
	xu = reshape(X(1:end-1), nvarpernode, N);
	x1 = xu(1:nstates,:);					% states of all nodes
	x2 = [x1(:,2:N) x1(vmx,1)];		% states of all subsequent nodes
	x2(1,N) = x2(1,N) + problem.speed * X(end);	% add horizontal translation (speed*duration) for cycle
	
    % controls of all subsequent nodes
    u2 = [xu(nstates+(1:ncontrols),2:N) xu(nstates+vmu,1)];    
	u1 = [xu(nstates+(1:ncontrols),1:N)]; 
    
	% compute the constraint violations c
	if strcmp(problem.discretization, 'BE')
        %c = h*gait2dem(x2,u2) - (x2-x1);
        c = h*gait2dem(x2,u1) - (x2-x1);
	elseif  strcmp(problem.discretization, 'ME')
        %c = h*gait2dem((x1+x2)/2,u2) - (x2-x1);
        c = h*gait2dem((x1+x2)/2,u1) - (x2-x1);
	else
		error('discretization %s unknown', problem.discretization);
    end	
    %=======================================================%
	
	% rearrange them into a column vector
	c = reshape(c, N*nstates, 1);

	% save norm of c for reporting
	problem.optlog.conval = norm(c);
	
end
%===========================================================================================
function J = conjacstructure();
	% returns structure of constraint Jacobian matrix
	global problem
	J = problem.Jpattern;
end	
%===========================================================================================
function J = conjac(X)
	% returns constraint Jacobian matrix or its sparsity structure
	global problem
	
	nvarpernode 	= problem.nvarpernode;
	ncon 			= problem.ncon;
	nvar 			= problem.nvar;
	N 				= problem.N;

	if (mod(N,2) ~= 0)
		error('Number of nodes N must be even (for quick Jacobian calculation).');
	end
	
	% J = spalloc(ncon,nvar,problem.Jnnz);
	J = problem.Jpattern;
	hh = 1e-7;				% finite difference
	c = confun(X);
	
	% do variables related to odd nodes, then even
	for k=0:1
		i_start = (k:2:N-1)*nvarpernode;
		% do all variables within one node separately (we can still improve on this!)
		for i=1:nvarpernode
			if (toc > 5)
				if (k==0)
					fprintf('Evaluating Jacobian for variable %d of %d, even nodes...\n',i, nvarpernode);
				else
					fprintf('Evaluating Jacobian for variable %d of %d, odd nodes...\n',i, nvarpernode);
				end
				tic;
			end
			index = i_start + i;
			Xsave = X(index);
			X(index) = X(index) + hh;
			derivatives = sparse(confun(X) - c)/hh;
			X(index) = Xsave;		
			% distribute the derivatives into columns of J according to sparsity pattern of J
			% it may be worthwhile doing this without the loop, see http://www.frontiernet.net/~dmschwarz/genops.html
			for col=index
				% if (toc > 1)
					% fprintf('var %d conjac column %d...\n', i, col);
					% tic;
				% end
				% the following line is extremely slow when Jpattern is not finalized yet!!
				J(:,col) = problem.Jpattern(:,col).*derivatives;
			end	
		end
	end
	
	% derivatives of constraints with respect to last element (dur) is done separately
	X(end) = X(end) + hh;
	J(:,end) = sparse((confun(X) - c)/hh);				% w.r.t. duration

	
end
%=================================================================
function [s] = ipoptmessage(info)

 	if info==0;  s = 'solved'; return; end;
 	if info==1;  s = 'solved to acceptable level'; return; end;
 	if info==2;  s = 'infeasible problem detected'; return; end;
 	if info==3;  s = 'search direction becomes too small'; return; end;
 	if info==4;  s = 'diverging iterates'; return; end;
 	if info==5;  s = 'user requested stop'; return; end;
%     
 	if info==-1;  s = 'maximum number of iterations exceeded'; return; end;
 	if info==-2;  s = 'restoration phase failed'; return; end;
 	if info==-3;  s = 'error in step computation'; return; end;
 	if info==-10;  s = 'not enough degrees of freedom'; return; end;
 	if info==-11;  s = 'invalid problem definition'; return; end;
 	if info==-12;  s = 'invalid option'; return; end;
 	if info==-13;  s = 'invalid number detected'; return; end;
%
 	if info==-100;  s = 'unrecoverable exception'; return; end;
 	if info==-101;  s = 'non-IPOPT exception thrown'; return; end;
 	if info==-102;  s = 'insufficient memory'; return; end;
 	if info==-199;  s = 'internal error'; return; end;

end
%================================================================================================
function savefile(X, filename, problem)
	% save this result in a Result structure on a file
	clear Result
	
	x = reshape(X(1:end-1),problem.nvarpernode,problem.N);
	Result.x = x(1:problem.nstates,:);
	Result.u = x(problem.nstates+(1:problem.ncontrols),:);
	Result.dur = X(end);
    
	times = Result.dur*(0:problem.N-1)/problem.N;
	if (nargin > 2)
		Result.problem = problem;
	end
	save(filename,'Result');
end
%===============================================================================================
function drawstick(x);

	R = [1:6 4];			% right stick points
	L = [2 7:10 8];			% left stick points
	xrange = [min(x(1,:))-0.5  , max(x(1,:))+0.5];
	yrange = [min(x(2,:))-1.2  , max(x(2,:))+0.5];
	plot(xrange,[0 0],'k');		% draw ground surface as a black line
	for i=1:size(x,2)
		hold on
		[~,~,d] = gait2dem(x(:,i), zeros(9,1));
		d = reshape(d',2,10)';
		plot(d(R,1),d(R,2),d(L,1),d(L,2));
		axis('equal');
		axis([xrange yrange]);
	end
	hold off;
end
%=========================================================================================
function report(X)
	global problem
	
	subplot(2,2,[1 3])
	x = reshape(X(1:end-1), problem.nvarpernode, problem.N);
	Nsticks = min(problem.N, 50);
	sticknodes = round(linspace(1, problem.N, Nsticks));
	x = x(1:problem.nstates, sticknodes);			% use only the states
	drawstick(x);
	title([num2str(problem.N) ' nodes']);
	if size(problem.optlog.data,1) > 1
		subplot(2,2,2);
		semilogy(problem.optlog.data(:,2:5));
		legend('total','tracking','effort','reg','Location','NorthWest');
		title('Objective');

		subplot(2,2,4);
		semilogy(problem.optlog.data(:,1)); 
		title('Norm of constraint violations');
		xlabel('Function evaluations');
	end
	pause(0.1);
	
	% save the file
	% savefile(X, 'optimsave.mat', problem);

end
%==============================================================================================
function matcompare(a,b);
	% compares two matrices and prints element that has greatest difference
	[maxerr,irow] = max(abs(a-b));
	[maxerr,icol] = max(maxerr);
	irow = irow(icol);
	fprintf('%8.5f at %d %d (%f vs. %f)\n', maxerr, irow, icol, full(a(irow,icol)),full(b(irow,icol)));
end
%=============================================================================================
function [problem] = loaddata(problem)
	% Load gait data from a .mat file created by preproc.m
    
    N = problem.N;
    
	fprintf('Reading subject movement data from file %s...\n', problem.gaitdata);
	load(problem.gaitdata);
	fprintf('Gait speed:    %9.4f m/s\n',gait.speed(1));  
	fprintf('Gait cycle:    %9.4f s\n',gait.dur(1));
	
	% Interpolating for the tracking cost function
	% generate sample times for each node in fraction of gait cycle
	Ncycle = size(gait.data,1)-1;		% number of data samples per gait cycle (data file includes the 100% gait cycle point!)
	tcycle = (0:Ncycle)/Ncycle; 		% time vector for gait data (in fraction of gait cycle)
	if (problem.symmetry)
		tresamp = (0:(2*N-1))/(2*N);	% we resample the gait cycle into 2N samples (because N is half gait cycle)
	else
		tresamp = (0:(N-1))/N;			% we resample the gait cycle into N samples (because N is gait cycle)
	end
	data = interp1(tcycle, gait.data, tresamp);
	
	% average the SD over the entire gait cycle
	sd = mean(gait.sd);
	for i=1:5
		if (sd(i) < 0.1)
			if (i<=3)
				disp('Warning: small SD values found for joint angle.  Using SD = 5 deg instead.');
				sd(i) = 5;
			else
				disp('Warning: small SD values found for GRF.  Using SD = 2% BW instead.');
				sd(i) = 2;
			end
		end
	end
	if (gait.dur(2) < 1e-4)
		disp('Warning: gait cycle duration had small SD.  Using SD = 5 ms instead.');
		gait.dur(2) = 0.005;
	end
						
	% Convert kinematic data to our gait2dem model generalized coordinates and to radians
	ihip = 1;
	iknee = 2;
	iankle = 3;
	igrfx = 4;
	igrfy = 5;
	kin_hip 	= data(:,ihip)*pi/180;
	kin_hip_sd 	= sd(:,ihip)*pi/180;
	kin_knee 	= -data(:,iknee)*pi/180;
	kin_knee_sd = sd(:,iknee)*pi/180;
	kin_ankle 	= data(:,iankle)*pi/180;
	kin_ankle_sd = sd(:,iankle)*pi/180;

	% Convert GRF to our coordinate system and to units of body weight
	grf_Y = 	data(:,igrfy);
	% grf_Y = grf_Y - min(grf_Y);				% ensure zero baseline
	grfscale = problem.weight / (2 * mean(grf_Y));
	grf_Y 		= grfscale * grf_Y;				% renormalize subject GRF to model weight so we can use it in the tracking cost function
	grf_Y_sd 	= grfscale * sd(:,igrfy);
	grf_X 		= grfscale * data(:,igrfx);
	grf_X_sd 	= grfscale * sd(:,igrfx);
		
	% store measurements in N x 10 matrix and SD in 1x10 matrix (right side columns 1-5, left side columns 6-10)
	data = [kin_hip kin_knee kin_ankle grf_X grf_Y];
	datasd = [kin_hip_sd kin_knee_sd kin_ankle_sd grf_X_sd grf_Y_sd];
	if problem.symmetry
		data = [data(1:N,:) data(N+1:2*N,:)];				% creates a bilateral half gait cycle from one gait cycle of unilateral data
		dur = gait.dur(1)/2;
		dursd = gait.dur(2)/2;
	else
		data = [data(1:N,:) data([N/2+1:N 1:N/2],:)];		% creates a bilateral whole gait cycle from one gait cycle of unilateral data 
		dur = gait.dur(1);
		dursd = gait.dur(2);
	end
	datasd = [datasd datasd];
	datasd = repmat(datasd,N,1);			% same SD value is replicated for all nodes

	% create output data
	% problem.speed = gait.speed(1);			% the average walking speed of the subject, prescribed to model
	problem.trackdata.av = data;
	problem.trackdata.sd = datasd;
	problem.trackdata.dur = dur;		 				% the subjects movement duration, used in cost function
	problem.trackdata.dursd = dursd;
end

%=============================================================================================

