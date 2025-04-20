# ROS2 Native Windows Install Guide (WIP)
=================================

This guide exists to install ROS2 binaries on Windows 10/11. While official documentation exclusively uses Chocolatey, a third-party package manager, to handle installation, this guide aims to automate the installation of dependencies not covered by Chocolatey. This does this with a hybrid approach by relying off native Windows 10/11 packages from Microsoft's 1st party Windows package manager: winget. The script should prompt users when their interaction is needed.

## Usage
1. Open **Command Prompt** as administrator (search "cmd" in the Start Menu, right-click → Run as administrator).
2. Run the following command:

```cmd
winget upgrade Microsoft.AppInstaller
```

3. Download the .bat installation script file in this folder on GitHub (InstallDeps.bat)
4. **Important:** Right click this file and select "Run as Administrator".

## User Interation Required
User intervention is only required for a few windows UAC requests, and the installation of Visual Studio Community 2019 along with GraphViz (RQt)

- VS 2019
This installer is NOT automated.
1. Once the GUI is opened for this installer, check the "Desktop development with C++" box
2. Then **UNCHECK C++ CMake tools!!!** This step is absolutely **CRUCIAL** as cmake will be sourced from somewhere else in a later step in this install script
3. A fair warning, this is a very large software package and will likely take quite a while to install (>15 mins at least).

- GraphViz (needed for RQt)
1. Once the GUI is opened for this installer, click next.
2. After agreeing to the licence, chose to either "Add Graphviz to the system PATH for all users" or for "current user". This is required for ROS2 to invoke Graphviz.

## Programs Installed
This scripts installs all necessary dependencies for ROS2 Humble from various places. The programs installed are as follows:
- Chocolatey [powershell command from official website]
- 7-zip (latest) [winget]
- Python 3.8 (3.8.3) [winget]
- Python PIP packages + many required dependencies (see https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html#install-dependencies) [pip/choco]
- Microsoft Visual C++ 2013, 2015-2022 Redistributable (x64 & x86) (latest) [winget]
- OpenSSL (1.1.1.2100) [choco]
- Visual Studio Community (2019) [microsoft website]
- OpenCV (3.4.6) [ROS2 GitHub archive]
- Cmake (latest) [winget]
- GraphViz (12.2.1) [official website]
- ROS2 Humble (latest) [official website]

## After Installation
After this process is complete, reboot your computer and your Windows machine is now ready to run ROS2! This guide was built around ROS2 Humble, but should work for various versions of ROS2 (untested). The version of ROS2 can be swapped out inside the batch script for easy access.
