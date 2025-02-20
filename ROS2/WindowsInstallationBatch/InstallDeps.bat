@echo off
cd "%userprofile%/Downloads"

REM USER SHOULD BE INSTRUCTED TO INSTALL APP INSTALLER FROM WINDOWS STORE BEFORE RUNNING THIS SCRIPT


REM Install Chocolatey
::Check user policy before attempting chocolatey install
::for loop to parse through execution policy var
for /f "delims=" %%E in ('powershell -Command "Get-ExecutionPolicy"') do set POLICY=%%E

if /I "%POLICY%"=="Restricted" (
    powershell -Command "Set-ExecutionPolicy AllSigned -Force"
    echo Execution policy changed to AllSigned for Chocolatey Install
)
else (
    echo Execution policy is already set to %POLICY%. Proceeding to Chocolatey Install
)
::Run offical chocolatey install command
Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))


REM INSTALL 7zip
winget install --id 7zip.7zip


REM PYTHON
winget install --id Python.Python.3.8 -v 3.8.3


REM VISUAL C++ LIBS
winget install --id Microsoft.VCRedist.2013.x64
winget install --id Microsoft.VCRedist.2013.x86
winget install --id Microsoft.VCRedist.2015+.x64
winget install --id Microsoft.VCRedist.2015+.x86


REM OPENSSL
choco install -y openssl --version 1.1.1.2100
setx /m OPENSSL_CONF "C:\Program Files\OpenSSL-Win64\bin\openssl.cfg"


REM OPENSSL WITHOUT CHOCOLATEY (DO NOT USE)
REM firedaemon is the only distributor of any v1.1.1 openssl window binaries INCLUDING ./lib/ and ./include/ that I could find
REM feel free to replace URL with an official/better binary of OpenSSLin the future if you want to replace chocolatey entirely
::mkdir "C:\Program Files\OpenSSL-Win64"
::set "opensslurl=https://download.firedaemon.com/FireDaemon-OpenSSL/openssl-1.1.1u.zip"
::set "zipfile=openssl-1.1.1u.zip"
::set "destination=C:\Program Files\OpenSSL-Win64"
::curl -o "%zipfile%" "%opensslurl%"
::if not exist "%zipfile%" (
::   echo Download failed. URL Unreachable. Exiting...
::    exit /b 1
::)
::7z x "%zipfile%" -o"%USERPROFILE%\Downloads\openssl-extracted"
::if not exist "%USERPROFILE%\Downloads\openssl-extracted" (
::    echo Extraction failed. Exiting...
::    exit /b 1
::)
::xcopy "%USERPROFILE%\Downloads\openssl-extracted\openssl-1.1\x64\*" "%destination%" /E /I /Y
::if not exist "%destination%" (
::    echo Copy operation failed. Exiting...
::    exit /b 1
::)
::setx /m OPENSSL_CONF "C:\Program Files\OpenSSL-Win64\bin\openssl.cfg"
::echo OpenSSL installation completed successfully.



REM VISUAL STUDIO
set "vsurl=https://aka.ms/vs/16/release/vs_community.exe"
set "vsfilename=program-installer.exe"

curl -o "%vsfilename%" "%vsurl%"
if not exist "%vsfilename%" (
    echo Download failed. URL unreachable. Exiting...
    exit /b 1
)

REM MUST MUST MUST Prompt user to uncheck "C++ CMake tools for Windows"
start vs_community.exe

REM to-do, wait for user to finish installing
echo Installation and setup completed.



REM Install OpenCV
set "opencvurl=https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip”
set "opencvzipfile=opencv-3.4.6-vc16.VS2019.zip"


curl -o "%opencvzipfile%" "%opencvurl%"
if not exist "%opencvzipfile%" (
    echo Download failed. Exiting...
    exit /b 1
)

7z x "%opencvzipfile%" -o"%USERPROFILE%\Downloads"
if not exist "%USERPROFILE%\Downloads" (
    echo Extraction failed. Exiting...
    exit /b 1
)

xcopy "%USERPROFILE%\Downloads\opencv\*" "C:\" /E /I /Y
if not exist "C:\opencv" (
    echo Copy operation failed. Exiting...
    exit /b 1
)


REM CMake
winget install --id Kitware.CMake