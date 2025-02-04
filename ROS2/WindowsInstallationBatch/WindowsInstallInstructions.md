ROS2 Native Windows Install Guide (WIP)
=================================
To Install ROS2 on Windows, official documentation gives installation instructions using Chocolatey, a 3rd party windows package manager.

This guide exists to install ROS2 distributions without Chocolatey, and relying off Microsoft's 1st party Windows package manager: winget. This way, all dependencies are installed directly on the user's system without the needed for a sandboxed package manager.

To start installing dependencies for ROS2 without Chocolatey, run the InstallDeps.bat file found in this repo. This will automatically install some dependencies with winget, and others by downloading binaries files from various sources and installing them in various places. The programs installed are as follows:
- Python 3.8 (3.8.3) [winget]
- Microsoft Visual C++ 2013 Redistributable (x64 & x86) (latest) [winget]
- Microsoft Visual C++ 2015-2022 Redistributable (x64 & x86) (latest) [winget]

After this process is complete, your machine is now ready to run a ROS2 installation. This can be found 
