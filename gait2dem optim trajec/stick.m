function stick(x, disp);
% stick.m: draw a stick figure of the model
% x can be a single state (column vector), or a series of column vectors, or the name of a MAT file containing such data
	
	% x can be a series of column vectors, or a file containing such data
	if isstr(x)
		load(x);
		x = x';
	end

	% was a horizontal displacement per frame requested?
	if (nargin < 2)
		disp = 0.0;
	end

	% plot the ground
	xrange = [min(x(1,:))-1  , max(x(1,:))+1+disp*size(x,2)];
	plot(xrange,[0 0],'k');
	hold on;

	R = [1:6 4];			% right stick points
	L = [2 7:10 8];			% left stick points

	tau = zeros(9,1);
	for i=1:size(x,2)
		[xdot,grf,d] = gait2dem(x(:,i),tau);
		d = reshape(d',2,10)';
		d(:,1) = d(:,1) + (i-1)*disp;			% add the horizontal displacement
		plot(d(R,1),d(R,2),'r',d(L,1),d(L,2),'g','LineWidth',2);
	end
	axis('equal');
	hold off;
end