# SimpleHandEye
<p align="center">
  <img src="doc/openfig.png" alt="image" width="60%" height="auto"/>
</p>

SimpleHandEye is an easy-to-use and hardware-independent Python package for finding the unknown transformation between the world and sensor coordinates of two independent pose tracking systems (e.g. the transformation between the camera and robot gripper or the camera and robot base). 

This tool is meant to be hardware independent, easy to use, and completely Pythonic and feature:

- Classes abstracting OpenCV `AX=YB` and `AX=XB` solvers
- A class for performing nonlinear optimization for minimizing parameters that minimize reprojection error (TODO) 
- Simple Python classes for querying ROS and ROS2 TF messages. 
- Simple Apriltag and Chessboard pose estimation classes.
- Classes for reading images from Intel Realsense (based on pyrealsense2), UVC USB cameras (TODO), and ROS/ROS2 image topics (TODO)

## Installation

Simply install through pip (TODO):

```bash
pip install simple-handeye
```

or clone and install as follows:

```bash
git clone https://github.com/Rooholla-KhorramBakht/SimpleHandEye.git
cd SimpleHandEye
pip install -e .
```
## How To Use?

Here, we provide some common applications of this package. However, this package may be used based any kind of pose sensing systems.


### Eye On Base Calibration
TODO

### Eye On Hand Calibration
TODO

### Vicon Marker to Object Extrinsic Calibration
TODO

### Vicon-Based Multi-Camera Extrinsic Calibration
The goal in this example is to find the extrinsic transformation between cameras installed on a robot/autonomous vehicle and the body coordinate frame (or any common coordinate frame). 

In this example, the first pose tracking system is the Vicon which tracks the pose of markers corresponding to the body frame and markers that are installed on an Apriltag board. The other pose sensor is the cameras of interest which continously track the pose of an Apriltag/chessboard. The overall setup is shown in the following image:
<p align="center">
  <img src="doc/multi_camera_extrinsics.png" alt="image" width="75%" height="auto"/>
</p>