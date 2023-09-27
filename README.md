# AARL Robot_Arm

## Summary
This repository contains files relevant to the design and operation of AARL's robot arm, which is capable of writing short words using a dry erase marker on a white board.  The purpose of this small scale robotics effort is two fold.  Firstly, it serves as an effective and engaging robotics demonstration piece at STEM community outreach events.  Secondly, it provides an example of the functionality that we expect from student robotics projects in the introductory robotics lab at Portland State University (PSU).  Although there are certainly many avenues through which our robot arm could continue to be improved, the basic framework provided in this repository is sufficient for its application as a scientific demonstration.

## Install Instructions
As a small scale robotics project, this repository contains information pertaining to both the mechanical design our robot arm and the code that is used to control it.  The mechanical design has been completed in Solidworks, while the high-level control code is written in Matlab and the low-level control code is written in C.  The most recent versions of any of these tools is sufficient for running this project.

## Hardware
Our robot arm makes use of AX12 and MX64 dynamexil servos, as well as the arbotix-m robocontroller from [Robotis](https://www.robotis.us/).

## Repository Organization
This repository has three main directories, all of which feature self-explanatory names.  Solidworks CAD documents are located in the CAD_Mechanical directory, while Matlab and C codes are located in the Code directory.  The Documentation directory contains datasheets and other information relevant to this project.
