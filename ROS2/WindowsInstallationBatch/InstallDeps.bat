@echo off
cd "%userprofile%/Downloads"

REM USER SHOULD BE INSTRUCTED TO INSTALL APP INSTALLER FROM WINDOWS STORE BEFORE RUNNING THIS SCRIPT


REM 7zip
winget install --id 7zip.7zip

REM PYTHON (official docs ask for this specific version)
winget install --id Python.Python.3.8 -v 3.8.3

REM VISUAL C++ LIBS
winget install --id Microsoft.VCRedist.2013.x64
winget install --id Microsoft.VCRedist.2013.x86
winget install --id Microsoft.VCRedist.2015+.x64
winget install --id Microsoft.VCRedist.2015+.x86

