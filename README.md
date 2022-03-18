# Keypoint Gait Generator

**Author:** Philipp Allgeuer

**Version:** 1.0

**Date:** 31/01/18

**Minimum requirements:** Matlab R2016b, Matlab/Octave Rotations Library v1.4.0

## General Overview
This Matlab software release implements the keypoint gait generator in a way
ideal for visualisation, development and testing. Instead of being implemented
in a real-time control loop fashion, instead the entire gait trajectory for both
legs is calculated at once and presented. The release also incorporates
functions for kinematic conversions between the joint, abstract and inverse pose
spaces, and implements leg tip points (referred to internally as foot floor
points, or FFP for short). Some required common utilities are also included.

## Getting Started
The Matlab/Octave Rotations Library can be obtained
[here](https://github.com/AIS-Bonn/matlab_octave_rotations_lib). Ensure that the
`RotationsLib` directory is on the Matlab path, either manually from the Current
Folder view, or using `LoadRotations.m`, as described in the library help.

Open Matlab and set the current directory as the KeypointGait directory. For the
code to run, Matlab must also be able to find the scripts/functions in the
subdirectories. As such, either manually add these to the Matlab path, or run:

    >> AddToPath

To then run the keypoint gait generator for the arms and legs, use:

    >> CalcArmAngle
    >> CalcKeypointTraj

Refer to the help of these two functions to see which arguments can be provided.

## Where To Get More Help?
If a look into the source code does not resolve an issue you have with the
software, then you can contact the author at the email address given in the
*Bugs and Improvements* section.

## Bugs and Improvements
I welcome all feedback, suggestions and bug reports (by opening a GitHub issue).
If you improve or fix anything about the library then I encourage you to let me
know so that the library can be improved for everyone!
