# ksp-krpc

## Project Description
The goal of this project is to automate various mission profiles using Kerbal Space Program as a simulation platform. 
This project will utilize the Kerbal Remote Procedure Calls mod to connect Python to KSP. Automation will uses various
aspects on control theory and is intended as an educational tool.

Current release includes a launch sequence using pid-controlled thrust limiter. Future releases will optimize the 
gravity turn and fine tune the PID controller. In addition, launch procedures will be designed to automate first stage 
booster landings and more.

## Before Using:
1. Ensure you have KSP version 1.7.3 (newer versions are not currently supported)
2. Download the kRPC mod and read relevant documentation https://krpc.github.io/krpc/getting-started.html
3. Download Python and relevant kRPC library (can download using pip)
