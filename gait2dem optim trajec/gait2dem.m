% function [xdot, grf, stick, moments] = gait2dem(x,tau);
%
% Full documentation is in the file gait2dem_reference.pdf
%
% Explicit differential equation for 2D musculoskeletal model : xdot = f(x,tau)
% with additional optional outputs.
%
% Input:
%	x			State of the model at N times (18 x N)
%	tau			Force/torque inputs at N times (9 x N)
%
% Output:
%   xdot		State derivatives (18 x N)
%	grf			(optional) 6 x N matrix, ground reactions (Fx, Fy, Mz) on right and left foot
%	stick		(optional) 20 x N matrix, x and y coordinates for 10 stick figure points
%	moments		(optional) 6 x N matrix with joint moments
%
% State variables are:
%	x(1)		global X coordinate of hip (m)
%	x(2)		global Y coordinate of hip (m)
%	x(3)		global orientation of trunk (rad), zero when standing upright
%	x(4)		right hip angle (rad), zero when standing, positive for flexion
%	x(5)		right knee angle (rad), zero when standing, positive for hyperextension
%	x(6)		right ankle angle (rad), zero when standing, positive for dorsiflexion
%	x(7-9)		left side hip, knee and ankle angles (rad)
%	x(10-18)	generalized velocities, i.e. time derivatives of x(1-9)
%
% Control inputs:
% u(1) 		external horizontal force applied to trunk center of mass
% u(2) 		external vertical force applied to trunk center of mass
% u(3) 		external moment applied to trunk center of mass
% u(4:6) 	joint moments applied at right hip, knee, ankle
% u(7:9) 	joint moments applied at leftt hip, knee, ankle
% Note: u(1), u(2), and u(3) should be zero if you want to control the model with joint torques and no external forces.
%
% The stick figure points are:
%	1		Trunk center of mass
%	2		Hip joint
%	3,4		Right knee and ankle
%	5,6		Right heel and toe
%	7-8		Left knee and ankle
% 	9-10	Left heel and toe	
