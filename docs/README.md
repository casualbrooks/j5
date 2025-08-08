# Documentation

This folder contains user and developer documentation for J5.

Contents
- BOM.md – Bill of Materials
- SAFETY.md – Safety and risk guidance
- CONTRIBUTING.md – Contribution guidelines
- LICENSES.md – Third‑party licenses

Getting started
- See the top‑level README.md for quick start.
- For ROS 2 build steps, see below.

ROS 2 on Windows (PowerShell)
1) Ensure ROS 2 (Humble/Iron) is installed and added to your environment. In PowerShell:
   - `$env:ChocolateyInstall` should be set if you used Chocolatey.
   - Run the shortcut "ROS 2 <distro> > ROS 2 Developer PowerShell" or dot‑source the setup script.
2) Build
   - `cd j5/ros_ws`
   - `colcon build --symlink-install`
   - `.\install\local_setup.ps1`
3) Run demos
   - Placeholder until packages are implemented.
