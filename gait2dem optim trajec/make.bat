@echo off

REM the following is needed to make sure LIB environment variable is set correcly for Microsoft C compiler
call "vcvars32"

REM now run the Makefile
nmake /nologo

REM give user a chance to see output
echo Hit ENTER to close window.
pause
