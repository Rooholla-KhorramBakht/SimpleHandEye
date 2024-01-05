% SimpleHandEye documentation master file, created by
% sphinx-quickstart on Sun Dec 24 10:30:56 2023.
% You can adapt this file completely to your liking, but it should at least
% contain the root `toctree` directive.

```{image} ../../doc/logo.png
:alt: logo
:class: bg-primary mb-1
:width: 80%
:align: center
```

<div style="height: 50px;"></div>

```{toctree}
:caption: 'Contents:'
:hidden:
:maxdepth: 2

tutorials/index
```

```{image} ../../doc/openfig.png
:alt: logo
:class: bg-primary mb-1
:width: 100%
:align: center
```

# Overview

SimpleHandEye is an easy-to-use and hardware-independent Python package for finding the unknown transformation between the world and sensor coordinates of two independent pose tracking systems (e.g. the transformation between the camera and robot gripper or the camera and robot base). 

This tool is meant to be hardware independent, easy to use, and completely Pythonic, and features:

- Classes abstracting OpenCV `AX=YB` and `AX=XB` solvers
- A class for performing nonlinear optimization for minimizing reprojection error (To be added) 
- Simple Python classes for querying ROS and ROS2 TF messages. 
- Simple visual trackers for Apriltags and Chessboard patterns. 
- Classes for reading images from Intel Realsense (based on pyrealsense2), UVC USB cameras, and ROS/ROS2 image topics.

## Installation

```bash
git clone https://github.com/Rooholla-KhorramBakht/SimpleHandEye.git
cd SimpleHandEye
pip install -e .
```
