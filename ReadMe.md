# Flexiv Robot VR Manipulation Project

## Project Overview
This project contains scripts for VR manipulation of a Flexiv robot. The primary file of interest is src/flexiv_speed_control.cpp, which connects the Flexiv robot to ROS and listens to a topic created in Unity. It acts as a middle layer to transfer messages to the robot. This code is written in C++ because the Flexiv requires high-frequency operations for real-time functionality, which C++ handles more efficiently than Python.

## Current Status

Unfortunately, I no longer have access to the Flexiv robot with which this project was developed. As such, I cannot verify whether it is still functioning correctly.

## Known Issues

There was an issue with the `CMakeLists.txt` file when I initially created this code, but it disappeared at some point, and I still do not know what caused this fix.

## Prerequisites
- flexiv_ros: You will need the `flexiv_ros` project foun [here](https://github.com/flexivrobotics/flexiv_ros).
- flexiv_rdk: You will also need the `flexiv_rdk` found [here](https://github.com/flexivrobotics/flexiv_rdk).

I suggest checking their GitHub repositories for the latest versions and installation instructions before using this project.

## Disclaimer

Due to the lack of access to the original hardware, this project may not work as expected. Please test it thoroughly in your environment and report any issues you encounter.
