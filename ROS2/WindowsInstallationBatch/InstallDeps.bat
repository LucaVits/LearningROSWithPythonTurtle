@echo off
setlocal ENABLEDELAYEDEXPANSION

:: EDIT THIS LINE BELOW TO CHANGE THE VERSION OF ROS2 INSTALLED
set "ROS2_VERSION=release-humble-20241205"
REM The URL and zip file variables below will update based on the version inputted above, no need to edit these unless the upstream file structure has changed
set "ROS2_ZIP=ros2-%ROS2_VERSION%-windows-release-amd64.zip"
set "ROS2_URL=https://github.com/ros2/ros2/releases/download/%ROS2_VERSION%/%ROS2_ZIP%"
set "ROS2_EXTRACTDIR=%ROS2_VERSION%-windows-release-amd64"

:: Enable a log
set LOGFILE=%USERPROFILE%\Downloads\ros2_install.log
(
  echo ==== ROS2 Install started at %DATE% %TIME% ====
  REM … rest of script …
  echo ==== Completed at %DATE% %TIME% ====
)>>"%LOGFILE%" 2>&1

:: Prerequisite Checks:

REM 1) Administrator privileges
net session >nul 2>&1
if %errorlevel% neq 0 (
  echo [ERROR] This script requires Administrator privileges. Please right‑click and “Run as administrator.”
  exit /b 1
)

REM 2) winget availability
where winget >nul 2>&1
if %errorlevel% neq 0 (
  echo [ERROR] winget not found. Install “App Installer” from the Microsoft Store.  
  exit /b 1
)

REM 3) curl availability
:: Unused becuase curl path is set to absolute path
:: where curl >nul 2>&1
:: if %errorlevel% neq 0 (
::   echo [ERROR] curl not found. Ensure you’re on Windows 10/11 and curl.exe is available. Also ensure \System32\curl.exe is in your system PATH
::   exit /b 1
:: )

:: Set Absolute Paths for system files used in installation

REM set working directory to Downloads to avoid System32 permission issues
cd "%USERPROFILE%\Downloads"

REM use absolute path for curl. Not sure why curl -o can't just be called on it's own
set "curlPath=%SystemRoot%\System32\curl.exe"

:: Check if system's curl executable exists here
if not exist "%curlPath%" (
    echo curl not found at %curlPath%. Check if you are on Windows 10/11. Exiting...
    exit /b 1



REM INSTALL 7-zip
winget install --id 7zip.7zip
if %ERRORLEVEL% neq 0 (
  echo 7-Zip install failed. Exiting...
  exit /b 1
)

REM use absolute path for 7zip. This is simpler than adding to PATH & forcing user to reboot:
set "zipPath=%ProgramFiles%\7-Zip\7z.exe"

:: Check if 7-Zip executable exists here
if not exist "%zipPath%" (
    echo 7-Zip not found at %zipPath%. Exiting...
    exit /b 1
)
REM NEEDS TO BE INSTRUCTED TO INSTALL APP INSTALLER FROM WINDOWS STORE BEFORE RUNNING THIS SCRIPT



REM INSTALL Chocolatey
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



REM INSTALL Python
winget install --id Python.Python.3.8 -v 3.8.3
py -3.8.3 -m pip install -U pip setuptools==59.6.0
py -3.8.3 -m pip install -U catkin_pkg cryptography empy importlib-metadata lark==1.1.1 lxml matplotlib netifaces numpy opencv-python PyQt5 pillow psutil pycairo pydot pyparsing==2.4.7 pyyaml rosdistro



REM INSTALL Visual C++ Libraries
winget install --id Microsoft.VCRedist.2013.x64
if %ERRORLEVEL% neq 0 (
  echo Microsoft Visual C++ 2013 (x64) install failed. Exiting...
  exit /b 1
)
winget install --id Microsoft.VCRedist.2013.x86
if %ERRORLEVEL% neq 0 (
  echo Microsoft Visual C++ 2013 (x86) install failed. Exiting...
  exit /b 1
)
winget install --id Microsoft.VCRedist.2015+.x64
if %ERRORLEVEL% neq 0 (
  echo Microsoft Visual C++ 2015+ (x64) install failed. Exiting...
  exit /b 1
)
winget install --id Microsoft.VCRedist.2015+.x86
if %ERRORLEVEL% neq 0 (
  echo Microsoft Visual C++ 2015+ (x86) install failed. Exiting...
  exit /b 1
)



REM INSTALL OpenSSL
choco install -y openssl --version 1.1.1.2100
if %ERRORLEVEL% neq 0 (
  echo OpenSSL install failed. Exiting...
  exit /b 1
)
setx /m OPENSSL_CONF "C:\Program Files\OpenSSL-Win64\bin\openssl.cfg"



REM INSTALL Visual Studio Community 2019
set "vsurl=https://aka.ms/vs/16/release/vs_community.exe"
set "vsfilename=program-installer.exe"


echo Downloading Visual Studio Community 2019...
curl -o "%vsfilename%" "%vsurl%"
if not exist "%vsfilename%" (
    echo Download failed. URL unreachable. Exiting...
    exit /b 1
)

REM MUST MUST MUST Prompt user to uncheck "C++ CMake tools for Windows"
	::currently show user what to check in installation readme. Could be explained clearer
start /wait vs_community.exe
echo VS Studio 19 install complete.



REM INSTALL OpenCV
set "opencvurl=https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.6-vc16.VS2019.zip”
set "opencvzipfile=opencv-3.4.6-vc16.VS2019.zip"

"%curlPath%" -o "%opencvzipfile%" "%opencvurl%"
if not exist "%opencvzipfile%" (
    echo Download failed. Exiting...
    exit /b 1
)

"%zipPath%" x "%opencvzipfile%" -o"%USERPROFILE%\Downloads"
if not exist "%USERPROFILE%\Downloads" (
    echo Extraction failed. Exiting...
    exit /b 1
)

xcopy "%USERPROFILE%\Downloads\opencv\*" "C:\" /E /I /Y
if not exist "C:\opencv" (
    echo Copy operation failed. Exiting...
    exit /b 1
)



REM INSTALL CMake
winget install --id Kitware.CMake
if %ERRORLEVEL% neq 0 (
  echo CMake install failed. Exiting...
  exit /b 1
)
set "CMAKE_BIN=C:\Program Files\CMake\bin"

:: Add CMake bin to PATH for the current session
setx /m PATH "%CMAKE_BIN%;%PATH%"

:: Verify that CMake is available
cmake --version
if %errorlevel% neq 0 (
    echo Error: CMake version check failed. Exiting...
    exit /b 1
)



REM INSTALL Random Choco Dependencies (From Offical ROS2 Repo)
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/asio.1.12.1.nupkg"
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/bullet.3.17.nupkg"
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/cunit.2.1.3.nupkg"
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/eigen.3.3.4.nupkg"
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/tinyxml-usestl.2.6.2.nupkg"
curl -O "https://github.com/ros2/choco-packages/releases/download/2022-03-15/tinyxml2.6.0.0.nupkg"

choco install -y -s %CD% asio cunit eigen tinyxml-usestl tinyxml2 bullet
if %ERRORLEVEL% neq 0 (
  echo Misc. dependencies from ROS2 website failed to install. Exiting...
  exit /b 1
)


REM INSTALL Qt5
set "qtDownloadUrl=https://download.qt.io/official_releases/qt/5.15/5.15.2/single/qt-everywhere-src-5.15.2.zip"
set "qtZipFile=qt-everywhere-src-5.15.2.zip"
set "qtExtractDir=Qt-5.15.2-Installer"

echo Downloading Qt 5.15.2 source code...
"%curlPath%" -o "%qtZipFile%" "%qtDownloadUrl%"
if not exist "%qtZipFile%" (
    echo Error: Failed to download Qt source code. Exiting...
    exit /b 1
)

echo Extracting Qt Installer from zip...
"%zipPath%" x "%qtZipFile%" -o "%CD%"
if not exist "%qtExtractDir%" (
    echo Error: Extraction failed. Exiting...
    exit /b 1
)

echo Running Qt5 Installer...
start /wait ./%qtExtractDir%/qt-everywhere-src-5.15.2.exe

setx /m Qt5_DIR C:\Qt\Qt5.12.12\5.12.12\msvc2017_64
setx /m QT_QPA_PLATFORM_PLUGIN_PATH C:\Qt\Qt5.12.12\5.12.12\msvc2017_64\plugins\platforms

echo Qt5 install complete



REM INSTALL Graphviz (needed for RQt)
set "graphvizurl=https://gitlab.com/api/v4/projects/4207231/packages/generic/graphviz-releases/12.2.1/windows_10_cmake_Release_graphviz-install-12.2.1-win64.exe”
set "graphvizfile=windows_10_cmake_Release_graphviz-install-12.2.1-win64.exe"

echo Downloading Graphviz 12.2.1...
"%curlPath%" -o "%graphviz%"
if not exist "%graphvizfile%" (
    echo Download failed. Exiting...
    exit /b 1
)

start /wait windows_10_cmake_Release_graphviz-install-12.2.1-win64.exe
echo GraphViz install complete.
 


REM Time to install ROS2 Humble!

echo Downloading Qt 5.15.2 source code...
"%curlPath%" -o "%ROS2_ZIP%" "%ROS2_URL%"
if not exist "%ROS2_ZIP%" (
    echo Error: Failed to download ROS2 Humble. Exiting...
    exit /b 1
)

echo Extracting ROS2 from zip...
"%zipPath%" x "%ROS2_ZIP%" -o "%CD%"
if not exist "%ROS2_EXTRACTDIR%" (
    echo Error: Extraction failed. Exiting...
    exit /b 1
)

echo Running ROS2 Installer...
start /wait ./%ROS2_EXTRACTDIR%/ros2-humble-20241205-windows-release-amd64.exe

echo ROS2 install complete

echo All Installations Successful!
REM This is a "press any button to continue" statement after the install is complete
pause

endlocal
