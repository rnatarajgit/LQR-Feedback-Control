# Makefile for creating the Matlab MEX function for the Gait2D model
# This Makefile was formatted for Microsoft's NMAKE.EXE

#  Use the mex.bat script from the Matlab installation that is first in the search path
MEX = "mex.bat"

# Microsoft Visual C++ compiler
CC = CL.exe					

MEXOPTS =		# use -g for faster compile and for debugging, remove -g for fastest execution

CFLAGS=/O2 /nologo

# the first target is the actual mex function
gait2dem.mexw32: gait2dem.c ..\gait2de.h ..\gait2de_al.obj
	$(MEX) $(MEXOPTS) gait2dem.c ..\gait2de_al.obj

# END OF MAKEFILE







