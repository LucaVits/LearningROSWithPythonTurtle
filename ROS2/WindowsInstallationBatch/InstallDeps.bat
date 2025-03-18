@echo off
setlocal

REM set user to downloads folder to avoid permission issues in System32
cd "%userprofile%/Downloads"

REM use absolute path for curl. Not sure why curl -o can't just be called on it's own
set "curlPath=%SystemRoot%\System32\curl.exe"
:: Check if system's curl executable exists here
if not exist "%curlPath%" (
    echo curl not found at %zipPath%. Check if you are on Windows 10/11. Exiting...
    exit /b 1
REM same with 7zip. This is simpler than adding to PATH & forcing user to reboot:
set "zipPath=%ProgramFiles%\7-Zip\7z.exe"
:: Check if 7-Zip executable exists here
if not exist "%zipPath%" (
    echo 7-Zip not found at %zipPath%. Exiting...
    exit /b 1
)

REM NEEDS TO BE INSTRUCTED TO INSTALL APP INSTALLER FROM WINDOWS STORE BEFORE RUNNING THIS SCRIPT


REM Install Chocolatey
::Check user policy before attempting chocolatey install
::for loop to parse through execution policy var (not working)
::Should not be required, powershell script for choco now has built in execution policy Bypass
for /f "delims=" %%i in ('powershell -Command "Get-ExecutionPolicy"') do set POLICY=%%i

if /I "%POLICY%"=="Restricted" (
    ::powershell -Command "Set-ExecutionPolicy AllSigned -Force"
		::DO NOT USE ABOVE! Unsafe and results in permission issues in this script
    echo Execution policy is restricted. Setting policy to be bypassed for the install of Chocolatey
) else (
    echo Execution policy is currently set to %POLICY%. Proceeding to Chocolatey Install...
)
::Run offical chocolatey install command
powershell -NoProfile -ExecutionPolicy Bypass -Command "[System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))"
pause


REM INSTALL 7zip
winget install --id 7zip.7zip


REM PYTHON
winget install --id Python.Python.3.8 -v 3.8.3
py -3.8.3 -m pip install -U pip setuptools==59.6.0
py -3.8.3 -m pip install -U catkin_pkg cryptography empy importlib-metadata lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5 pillow psutil pycairo pydot pyparsing==2.4.7 pyyaml rosdistro


REM VISUAL C++ LIBS
winget install --id Microsoft.VCRedist.2013.x64
winget install --id Microsoft.VCRedist.2013.x86
winget install --id Microsoft.VCRedist.2015+.x64
winget install --id Microsoft.VCRedist.2015+.x86


REM OPENSSL
choco install -y openssl --version 1.1.1.2100
setx /m OPENSSL_CONF "C:\Program Files\OpenSSL-Win64\bin\openssl.cfg"


REM VISUAL STUDIO
set "vsurl=https://aka.ms/vs/16/release/vs_community.exe"
set "vsfilename=program-installer.exe"

curl -o "%vsfilename%" "%vsurl%"
if not exist "%vsfilename%" (
    echo Download failed. URL unreachable. Exiting...
    exit /b 1
)

REM MUST MUST MUST Prompt user to uncheck "C++ CMake tools for Windows"
	::currently show user what to check in installation readme. Could be explained clearer
start /wait vs_community.exe
echo VS Studio 19 install complete.



REM Install OpenCV
set "opencvurl=https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip”
set "opencvzipfile=opencv-3.4.6-vc16.VS2019.zip"

"%curlPath%" -o "%opencvzipfile%" "%opencvurl%"
if not exist "%opencvzipfile%" (
    echo Download failed. Exiting...
    exit /b 1
)

"%7zipPath%" x "%opencvzipfile%" -o"%USERPROFILE%\Downloads"
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
set "CMAKE_BIN=C:\Program Files\CMake\bin"
:: Add CMake bin to PATH for the current session
setx /m PATH "%CMAKE_BIN%;%PATH%"
:: Verify that CMake is available, TO DO - add check if this returns an error
cmake --version


REM Random Choco Dependencies (From Offical ROS2 Repo)
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/asio.1.12.1.nupkg"
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/bullet.3.17.nupkg"
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/cunit.2.1.3.nupkg"
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/eigen.3.3.4.nupkg"
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/tinyxml-usestl.2.6.2.nupkg"
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/tinyxml2.6.0.0.nupkg"

choco install -y -s %CD% asio cunit eigen tinyxml-usestl tinyxml2 bullet


endlocal