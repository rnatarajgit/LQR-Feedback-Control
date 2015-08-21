% PROGRAM DESCRIPTION: Script to create optimal feedback (LQR) controllers 
% for full gait-cycle torque control of 2-D walking model (gait2dem). 
% Optimal feedback gains computed according to specific single control 
% parameter (Q/R ratio, i.e., tracking/effort) and optimal open-loop 
% control and state trajectory results over half gait-cycle. 
% by Raviraj Nataraj, 20150426
% Human Motion Control Lab (PI: van den Bogert) Cleveland State University

function A1_LQR_create_controller

for nodes = 50 %[150 250 300 350]; %[50, 100, 200, 400]     % number of nodes in half gait-cycle, data sets collected/outputted in respective folders
    load(['gait2dem optim trajec\result006_BE',num2str(nodes),'.mat']);     % loading results for optimal open-loop state, torque trajectories over half gait-cycle
    
    for QR_ratio = [2e-1 5e-1]; %[1e-3 1e-2 1e-1 1e0 2e0 5e0 1e1 2e1 5e1 1e2] % controller design parameter (Q/R ratio)
        
        tic;
        QR_ratio
        [Result_optfb] = optimal_feedback(Result,QR_ratio,1e3);         % solve for optimal feedback results from optimal open-loop results, desired Q/R performance, maximum solution iterations
        save(['optim feedback gains\BE',num2str(nodes),'\Results_QR',num2str(QR_ratio),'.mat'], 'Result_optfb');       % saving individual optimal feedback controller results to mat-file
        toc
        
    end
    
end

end
%=============================================================================================
% Solving A,B,K for Optimal Feedback Control (Linear Quadratic Regulator)   
function [Result_optfb] = optimal_feedback(Result, QR_ratio, maxit)  

    % Extracting select results from optimal open-loop simulation
    J = Result.problem.J; % Sparse constraint Jacobian
    X = Result.x; % State vectors
    U = Result.u; % Control vectors
    nstates = Result.problem.nstates; % Number of states
    ncontrols = Result.problem.ncontrols; % Number of controls
    N = Result.problem.N; % Number of time-nodes

    % Extracting A,B matrices from constraint Jacobian
    [A,B] = extractAB(J,nstates,ncontrols,N);

    % Creating full gait-cycle X,U,A,B from half gait-cycle data 
	displacement = Result.dur * Result.problem.speed;   % displacement in half gait cycle
    [Xf,Uf,Af,Bf] = create_fullgait(X,U,A,B,displacement);
      
    % Assigning weighting matrices for objective function terms (tracking, effort)
    Bfp = Bf(:,4:9,:); Ufp = Uf(4:9,:); % retain only 6 joint torques (omit 3 hip/torso forces)
    for i = 1:length(Xf);
        Q(:,:,i) = QR_ratio*diag(1./std(Xf').^2); % Weighting matrix for states (tracking)
        R(:,:,i) = diag(1./std(Ufp').^2); % Weighting matrix for controls (effort)        
    end    
    
    % Running discrete, periodic Riccati equation solver to compute K (optimal feedback gain matrix)
    [Xric, k, residuals] = dpre(Af,Bfp,Q,R,[],[],[],maxit); % [X,K] = dpre(A,B,Q,R,S,E,tol,maxit)
    
    % Storing select data structures for optimal feedback results
    Result_optfb.x = Xf;
    Result_optfb.u = Uf;
    Result_optfb.A = Af;
    Result_optfb.B = Bf;    
    Result_optfb.Q = Q;
    Result_optfb.R = R;
    Result_optfb.k = k;
    Result_optfb.xric = Xric;
    Result_optfb.resid = residuals;
    Result_optfb.problem.N = length(Xf);
    Result_optfb.problem.dur = Result.dur*length(Xf)/length(X);
    Result_optfb.problem.nstates = size(Af,2);
    Result_optfb.problem.ncontrols = size(Bf,2);
    Result_optfb.problem.speed = Result.problem.speed;
    
end
%=============================================================================================
% Getting out A(plant), B(controller inputs) matrices from Jacobian
function [A,B] = extractAB(J,nstates,ncontrols,N) 
    A = zeros(nstates,nstates,N);
    B = zeros(nstates,ncontrols,N);
    for i = 1:N
        rows = (i-1)*nstates + (1:nstates);
        x1cols = (i-1)*(nstates+ncontrols) + (1:nstates);
        u1cols = (max(x1cols)+1):(max(x1cols)+ncontrols);        
        if i < N
            x2cols = (max(u1cols)+1):(max(u1cols)+nstates);
        elseif i == N
            sis = [1:3 7:9 4:6 10:12 16:18 13:15]; % symmetry index parameters for states (1:18)
            x2cols = sis;            
        end
        A(:,:,i) = -inv(J(rows,x2cols)) * J(rows,x1cols); 
        B(:,:,i) = -inv(J(rows,x2cols)) * J(rows,u1cols);
    end
end
%=============================================================================================
% Creating full gait-cycle data set from half gait-cycle data and assuming right-left symmetry
function [Xf,Uf,Af,Bf] = create_fullgait(X,U,A,B,displacement)

    nstates = size(A,1); % Number of states
    ncontrols = size(B,2); % Number of controls
    N = size(A,3); % Number of time-nodes

    % Initializing full gait-cycle size and first half gait-cycle data for X, A, and B
    Xf = zeros(nstates,2*N); Uf = zeros(ncontrols,2*N); Af = zeros(nstates,nstates,2*N); Bf = zeros(nstates,ncontrols,2*N);
    Xf(:,1:N) = X;
    Uf(:,1:N) = U;
    Af(:,:,1:N) = A;
    Bf(:,:,1:N) = B;

    % Filling-out second half gait-cycle data for X, A, and B based on right-left leg symmetry
	sis = [1:3 7:9 4:6 10:12 16:18 13:15]; % symmetry index parameters for states
	sic = [1:3 7:9 4:6]; % symmetry index parameters for controls 
    Xf(:,(N+1):2*N) = X(sis,1:N); 
    Xf(1,(N+1):2*N) = X(1,1:N) + displacement; % Adding global x progress from first half gait-cycle
    Uf(:,(N+1):2*N) = U(sic,1:N);
    for k = (N+1):2*N
        Af(:,:,k) = A(sis,sis,k-N);
        Bf(:,:,k) = B(sis,sic,k-N);
    end

end
%=============================================================================================
function [X,K,RES] = dpre(A,B,Q,R,S,E,tol,maxit)
    %DPRE Discrete-time Periodic Riccati Equation
    %  [X,K]=DPRE(A,B,Q,R,S,E) computes the unique stabilizing solution X{k},
    %  k = 1:P, of the discrete-time periodic Riccati equation
    %
    %   E{k}'X{k}E{k} = A{k}'X{k+1}A{k} - (A{k}'X{k+1}B{k} + S{k})*...
    %                 (B{k}'X{k+1}B{k} + R{k})\(A{k}'X{k+1}B{k} + S{k})' + Q{k}
    %
    %  When omitted, R, S and E are set to the default values R{k}=I, S{k}=0,
    %  and E{k}=I. Beside the solution X{k}, DPRE also returns the gain matrix
    %
    %   K{k} = (B{k}'X{k+1}B{k} + R{k})\(B{k}'X{k+1}A{k} + S{k}'),
    %
    %  All input matrices have to be multidimensional arrays, like matrix
    %  A(N,N,P) and B(N,R,P). Output matrices are also multidimensional arrays
    %  in the size of X(N,N,P) and K(R,N,P).
    %
    %  [X,K]=DPRE(A,B,Q,R,S,E,TOL) specifies the tolerance of the cyclic qz
    %  method. If tol is [] then DPRE uses the default, 1e-6.
    %
    %  [X,K]=DPRE(A,B,Q,R,S,E,TOL,MAXIT) specifies the maximum number of
    %  iterations. If MAXIT is [] then DPRE uses the default, 1000. Warning is
    %  given when the maximum iterations is reached.
    %
    %  See also DARE.

    %  This version uses a cyclic qz method, see references.

    %  References:
    %    [1] J.J. Hench and A.J. Laub, Numerical solution of the discrete-time
    %        periodic Riccati equation, IEEE Trans. on automatic control, 1994
    %    [2] Varga, A., On solving discrete-time periodic Riccati equations,
    %        Proc. of 16th IFAC World Congress 2005, Prague, July 2005.

    %  Ivo Houtzager
    %
    %  Delft Center of Systems and Control
    %  The Netherlands, 2007


    % assign default values to unspecified parameters
    [m,n,p] = size(A);
    [mb,r,pb] = size(B);
    if (nargin < 8) || isempty(maxit)
        maxit = 1000;
    end
    if (nargin < 6) || isempty(E)
        E = zeros(m,n,p);
        for i = 1:p
            E(:,:,i) = eye(m,n);
        end
    end
    if (nargin < 5) || isempty(S)
        S = zeros(mb,r,pb);
    end
    if (nargin < 4) || isempty(R)
        R = zeros(r,r,pb);
        for i = 1:pb
            R(:,:,i) = eye(r);
        end
    end
    if (nargin < 7) || isempty(tol)
        tol = 1e-6;
    end

    % check input arguments
    if nargin < 2
        error('DPRE requires at least three input arguments')
    end
    [mq,nq,pq] = size(Q);
    [mr,nr,pr] = size(R);
    [ms,ns,ps] = size(S);
    [me,ne,pe] = size(E);
    if ~isequal(p,pb,pq,pr,ps,pe)
        error('The number of periods must be the same for A, B, Q, R, S and E.')
    end
    if ~isequal(m,me,mq,mb)
        error('The number of rows of matrix A, B, E and Q must be the same size.')
    end
    if ~isequal(n,ne,nq)
        error('The number of columns of matrix A, E and Q must be the same size.')
    end
    if ~isequal(mb,ms)
        error('The number of rows of matrix B and S must be the same size.')
    end
    if ~isequal(r,ns)
        error('The number of columns of matrix B and S must be the same size.')
    end
    if ~isequal(r,nr)
        error('The number of columns of matrix R must be the same size as the number of columns of matrix B.')
    end
    if ~isequal(r,mr)
        error('The number of rows of matrix R must be the same size as the number of columns of matrix B.')
    end

    % allocate matrices
    M = zeros(2*n+r,2*n+r,p);
    L = zeros(2*n+r,2*n+r,p);
    V = zeros(2*n+r,2*n+r,p);
    Z = zeros(2*n+r,2*n+r,p);
    Y = zeros(2*n+r,2*n+r,p);
    T = zeros(n+r,n,p);

    % build the periodic matrix pairs
    for i = 1:p
        L(:,:,i) = [A(:,:,i) zeros(n) B(:,:,i);
            -Q(:,:,i) E(:,:,i) -S(:,:,i);
            S(:,:,i)' zeros(r,n) R(:,:,i)];
        M(:,:,i) = [E(:,:,i) zeros(n,n+r);
            zeros(n) A(:,:,i)' zeros(n,r);
            zeros(r,n) -B(:,:,i)' zeros(r)];
        V(:,:,i) = eye(2*n+r);
        Z(:,:,i) = eye(2*n+r);
    end

    % cyclic qz decomposition
    k = 1;
    ok = true;
    res = ones(1,p);
    while ok == true && k <= maxit
        for j = 1:p
            % QR decomposition
            [Q,R] = qr(M(:,:,j));
            V(:,:,j) = V(:,:,j)*Q';
            M(:,:,j) = Q'*M(:,:,j);

            % RQ decomposition
            [Q,R] = qr(fliplr((Q'*L(:,:,j))'));
            Q = fliplr(Q);
            R = fliplr(flipud(R))';

            Z(:,:,j) = Z(:,:,j)*Q;
            Y(:,:,j) = Q;
            L(:,:,j) = R;
        end

        for j = 1:p
            if j == p
                M(:,:,p) = M(:,:,p)*Y(:,:,1);
            else
                M(:,:,j) = M(:,:,j)*Y(:,:,j+1);
            end

            T1 = Z(n+1:2*n+r,1:n,j)/Z(1:n,1:n,j);

            % calculate residue
            res(j) = norm(T1 - T(:,:,j));
            T(:,:,j) = T1;
        end

        if all(res <= tol)
            ok = false;
        end

        k = k + 1;
    end

    % return warning if k exceeds maxit
    if ok == true
        warning('DPRE:maxIt','Maximum number of iterations exceeded.')
    end

    % retrieve K and X matrices
    X = zeros(n,n,p);
    K = zeros(r,n,p);
    for i = 1:p
        X(:,:,i) = T(1:n,1:n,i);
        K(:,:,i) = -T(n+1:n+r,1:n,i);
    end
    
    % outputting final residual values
    RES = res;

end
%=============================================================================================