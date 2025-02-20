ROS2 Native Windows Install Guide (WIP)
=================================
To Install ROS2 on Windows, official documentation gives installation instructions using Chocolatey, a 3rd party windows package manager.

This guide exists to install ROS2 distributions with some Chocolatey packages & some native packages by relying off Microsoft's 1st party Windows package manager: winget. The script should prompt users when their interaction is needed.

To start installing dependencies for ROS2 without Chocolatey, run the InstallDeps.bat file found in this repo. This will automatically install some dependencies with winget, and others by downloading binaries files from various sources and installing them in various places. The programs installed are as follows:
- Chocolatey [from website]
- 7-zip (latest) [winget]
- Python 3.8 (3.8.3) [winget]
- Microsoft Visual C++ 2013 Redistributable (x64 & x86) (latest) [winget]
- Microsoft Visual C++ 2015-2022 Redistributable (x64 & x86) (latest) [winget]
- OpenSSL (1.1.1.2100) [choco]
- Visual Studio Community (2019) [microsoft website]
NOTE - Must check the following boxes:
https://docs.ros.org/en/humble/_images/windows-vs-studio-install.png
- OpenCV (3.4.6) [ROS2 github archive]
- Cmake (latest) [winget]



After this process is complete, your machine is now ready to run a ROS2 installation. This can be found 
