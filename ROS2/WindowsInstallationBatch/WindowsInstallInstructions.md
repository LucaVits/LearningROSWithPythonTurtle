ROS2 Native Windows Install Guide (WIP)
=================================
To Install ROS2 on Windows, official documentation gives installation instructions using Chocolatey, a 3rd party windows package manager.

This guide exists to install ROS2 distributions with some Chocolatey packages & some native packages by relying off Microsoft's 1st party Windows package manager: winget. The script should prompt users when their interaction is needed.


To start installing dependencies for ROS2, The Windows package manager (winget) needs to be installed.
simply open a command promt as administrator, and run the following:

```
winget upgrade Microsoft.AppInstaller
```

Next, download the InstallDeps.bat file found in this repo. Then right click, and RUN AS ADMINISTRATOR!
User intervention is only required for windows UAC requests, Visual Studio Community 2019, and GraphViz (RQt)

- VS 2019
check the ___ box. Then **UNCHECK C++ CMake tools!!!** This step is absolutely **CRUCIAL** as cmake will be sourced from somewhere else in a later step in this install script

- GraphViz (needed for RQt)
The installer needs to be clicked through. After agreeing to the licence, chose to either "Add Graphviz to the system PATH for all users" or for "current user". This is required for ROS2 to invoke Graphviz.


This will automatically install all necessary dependencies for ROS2 Humble from various places. The programs installed are as follows:
- Chocolatey [pwsh cmd from website]
- 7-zip (latest) [winget]
- Python 3.8 (3.8.3) [winget]
- Many required dependencies (see https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html#install-dependencies) [pip/choco]
- Microsoft Visual C++ 2013 Redistributable (x64 & x86) (latest) [winget]
- Microsoft Visual C++ 2015-2022 Redistributable (x64 & x86) (latest) [winget]
- OpenSSL (1.1.1.2100) [choco]
- Visual Studio Community (2019) [microsoft website]
- OpenCV (3.4.6) [ROS2 github archive]
- Cmake (latest) [winget]
- GraphViz (12.2.1) [website]
- ROS2 Humble (latest) [website]

After this process is complete, your machine is now ready to run a ROS2 installation. This guide was built around ROS2 Humble, but should work for various versions of ROS2 (untested). The version of ROS2 can be swapped out inside the batch script for easy access.