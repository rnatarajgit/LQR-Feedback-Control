/*=================================================================
 *
 * gait2dem.c
 *
 * Explicit differential equation for 2D musculoskeletal model : dx/dt = f(x,u)
 * This is the source code for the MEX function gait2dem.mexw32
 * The musculoskeletal model is documented in the file gait2dem_reference.docx
 * The function documentation is in gait2dem.m
 *
 * Author: Ton van den Bogert, Cleveland State University
 *
 *=================================================================*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "mex.h"
#include "gait2de.h"

// size of the model
#define NSTATES 2*NDOF		// number of system state variables, positions and velocities

// M_PI is known in gcc but not in Visual C++ 2008
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

// global variables
static int initialized = 0;
static param_struct param;				// contains model parameters that must be shared with Autolev code

// Other parameters	
static double par_JointK1, par_JointK2, par_JointB;		// passive joint stiffness and damping parameters
static double par_MinAngle[NMOM], par_MaxAngle[NMOM];	// passive joint range of motion

// ===================================================================================
// SetParameters: set all model parameters
// ===================================================================================
void SetParameters() {
	int i,j;
	
	// multibody model parameters, from Winter book for 75 kg body mass and 1.8 m body height
	param.TrunkMass 	= 50.8500;
	param.ThighMass 	= 7.5000;
	param.ShankMass 	= 3.4875;
	param.FootMass 		= 1.0875;

	param.TrunkInertia 	= 3.1777;
	param.ThighInertia 	= 0.1522;
	param.ShankInertia 	= 0.0624;
	param.FootInertia 	= 0.0184;

	param.TrunkCMy 		= 0.3155;
	param.ThighCMy 		= -0.1910;
	param.ShankCMy 		= -0.1917;
	param.FootCMx		= 0.0768;
	param.FootCMy 		= -0.0351;
	param.ThighLen 		= 0.4410;
	param.ShankLen 		= 0.4428;
	
	// contact model parameters
	param.ContactHeelX	= -0.06;			// X coordinate of heel contact point
	param.ContactToeX	= 0.15;				// X coordinate of toe contact point
	param.ContactY		= -0.07;			// Y coordinate of both contact points
	param.ContactStiff	= 5e7;				// ground contact stiffness, N/m^3
	param.ContactDamp	= 0.85;				// ground contact damping, s/m
	param.ContactFric	= 1.0;         		// friction coefficient
	param.ContactV0		= 0.01;				// velocity constant (m/s), for |ve| = vc -> |fx| = 0.4621*c*fy
	
	// passive joint moment parameters
	par_JointK1			= 1;				// overall joint stiffness (Nm/rad)
	par_JointK2			= 10000;			// stiffness at joint limits (Nm/rad^2)
	par_JointB			=  2;				// joint damping (Nms/rad), exists always
	par_MinAngle[0] 	=  -50;				// Rhip
	par_MaxAngle[0] 	=  160;		
	par_MinAngle[1] 	= -160;				// Rknee
	par_MaxAngle[1] 	=   10;
	par_MinAngle[2] 	=  -90;				// Rankle
	par_MaxAngle[2] 	=   60;
	// copy the right side range of motion into the left side
	for (i=0; i<NMOM/2; i++) {
		j = i + NMOM/2;
		par_MinAngle[j] = par_MinAngle[i];
		par_MaxAngle[j] = par_MaxAngle[i];
	}
	
	// Preprocessing and error checking
	for (i=0; i<NMOM; i++) {
		par_MinAngle[i] = par_MinAngle[i]*M_PI/180.0;
		par_MaxAngle[i] = par_MaxAngle[i]*M_PI/180.0;
		if (par_MinAngle[i] > par_MaxAngle[i]) {
			printf("Error in joint %d\n", i);
			mexErrMsgTxt("Max angle must be greater than min angle.");
		}
	}
}

// =========================================================================
// mexFunction: this is the actual MEX function interface
// =========================================================================
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {

	// working variables
	int i, j, k, nrows, ncols, iframe, nframes;
	double d, ang, angvel;

	// multibody dynamics variables
	double *q, *qd;
	double qdd[NDOF];
	double mom[NDOF];
	double stick[2*NSTICK];
	double grf[6];
	
	// MEX function pointers to inputs and outputs
	double *x, *xdot, *tau;
	double *mom_out;
	double *grf_out, *stick_out;
	
	// Initialize the model, if needed
	if (initialized != 1959) {
		printf("********************************************************************\n");
		printf("*           GAIT2DEM -- Moment-driven 7-link biped                 *\n");  
		printf("*         Version 1.0 -- Compiled: "__DATE__" "__TIME__"            *\n"); 
		printf("*              Licensed for non-commercial use only                *\n");
		printf("*              (c) 2013 Cleveland State University                 *\n");
		printf("*                                                                  *\n"); 
		printf("*  This software, and all associated files, may be distributed     *\n"); 
		printf("*  freely without restriction, provided that this text is not      *\n"); 
		printf("*  altered.  The latest version may be downloaded from             *\n");
        printf("*  http://hmc.csuohio.edu.                                 *\n"); 
		printf("*                                                                  *\n"); 
		printf("* THERE IS NO WARRANTY FOR THE PROGRAM, TO THE EXTENT PERMITTED    *\n"); 
		printf("* BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE   *\n"); 
		printf("* COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE PROGRAM       *\n");
		printf("* \"AS IS\" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR        *\n");
		printf("* IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES   *\n");
		printf("* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE     *\n");
		printf("* ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS  *\n");
		printf("* WITH YOU. SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE     *\n");
		printf("* COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.           *\n");
		printf("********************************************************************\n");
		printf("Initializing model...\n");
		SetParameters();
		initialized = 1959;
	}
	
	if (nrhs<2) {
		mexErrMsgTxt("gait2dem: must have 2 inputs.");
	}

	// State of model is the first input (required)
	nrows = mxGetM(prhs[0]);
	ncols = mxGetN(prhs[0]);
	if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0]) )
		mexErrMsgTxt("gait2dem: Incorrect type for state vector x, must be double.");
	if (nrows != NSTATES)
		mexErrMsgTxt("gait2dem: Incorrect size for state vector x, must have 18 rows.");
	x = mxGetPr(prhs[0]);
	nframes = ncols;
		
	// Controls are second input (required)
	nrows = mxGetM(prhs[1]);
	ncols = mxGetN(prhs[1]);
	if (!mxIsDouble(prhs[1]) || mxIsComplex(prhs[1]) ) mexErrMsgTxt("gait2dem: Incorrect type for tau, must be double.");
	if (nrows != NDOF) mexErrMsgTxt("gait2dem: Incorrect size for tau, must have 9 rows.");
	if (ncols != nframes) mexErrMsgTxt("gait2dem: tau must have same number of columns as x.");
	tau = mxGetPr(prhs[1]);
	
	// Create matrix for the xdot output of the MEX function
	plhs[0] = mxCreateDoubleMatrix(NSTATES, nframes, mxREAL);
	xdot = mxGetPr(plhs[0]);

	// Create matrix for the optional GRF output of the MEX function
	if (nlhs > 1) {
		plhs[1] = mxCreateDoubleMatrix(6, nframes, mxREAL);
		grf_out = mxGetPr(plhs[1]);	
	}
	
	// Create matrix for the optional stick output of the MEX function
	if (nlhs > 2) {
		plhs[2] = mxCreateDoubleMatrix(NSTICK*2, nframes, mxREAL);
		stick_out = mxGetPr(plhs[2]);	
	}

	// Create matrix for the optional joint moment output of the MEX function
	if (nlhs > 3) {
		plhs[3] = mxCreateDoubleMatrix(NMOM, nframes, mxREAL);
		mom_out = mxGetPr(plhs[3]);
	}
	
	for (iframe=0; iframe < nframes; iframe++) {

		// Compute the generalized forces for input to Autolev code
		for (i=0; i<NDOF; i++) {
			mom[i] = tau[i];
			
			// if this DOF is a joint angle, add passive joint moments
			if (i>2) {
				ang = x[i];											// joint angle is one of the state variables
				angvel = x[NDOF+i];									// the corresponding angular velocity
				
				// is angle above upper limit of ROM?
				d = ang - par_MaxAngle[i-3];
				if (d > 0.0) {	
					mom[i] = mom[i] - par_JointK2 * d*d;
				}
				
				// is angle below lower limit of ROM?
				d = ang - par_MinAngle[i-3];
				if (d < 0.0) {
					mom[i] = mom[i] + par_JointK2 * d*d;
				}
				
				// add a small amount of damping and overall linear stiffness term
				mom[i] = mom[i] - par_JointB * angvel - par_JointK1 * ang;
					
				// copy joint moments to the MEX function output variable, if needed
				if (nlhs > 3) *(mom_out++) = mom[i];
			}			
		}
		
		// Call the C function that was generated by Autolev
		q = &x[0];
		qd = &x[NDOF];	
		gait2d_al(&param, q, qd, qdd, mom, grf, stick);
		
		// for (i=0; i<9; i++) printf("mom(%d): %9.4f\n", i, mom[i]);
		// for (i=0; i<9; i++) printf("qdd(%d): %9.4f\n", i, qdd[i]);
		// for (i=0; i<6; i++) printf("grf(%d): %9.4f\n", i, grf[i]);
		
		// copy ground reaction forces to the MEX function output variable, if needed
		if (nlhs > 1) for (i=0; i<6; i++) *(grf_out++) = grf[i];

		// copy stick figure data to the MEX function output variable, if needed
		if (nlhs > 2) for (i=0; i<2*NSTICK; i++) *(stick_out++) = stick[i];

		// copy the elements of xdot into the MEX function output, columnwise
		for (i=0; i<NDOF; i++) *(xdot++) = x[NDOF+i];		// the first NDOF rows are: qdot = dq/dt
		for (i=0; i<NDOF; i++) *(xdot++) = qdd[i];			// the next NDOF rows are the equations of motion from Autolev
	
		// advance the input pointers to the next frame
		x = x + NSTATES;
		tau = tau + NDOF;
	
	}
	return;
}
