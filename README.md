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

#### Tracking $\mathbf{{}^{body}T_{marker}}$ :
To track the relative pose between the marker frame installed on the board and the body frame installed on the robot, we use the ROS2/ROS1 interface to read the TF messages published by vicon-bridge node running in a separate terminal. Instantiate the pose listener as follows:

**For ROS2:**
```python
from SimpleHandEye.interfaces.utils import addFoxyPath
addFoxyPath('/opt/ros/foxy')
from SimpleHandEye.interfaces.ros2 import ROS2TFInterface
import rclpy

rclpy.init()    
marker_pose_listener = ROS2TFInterface('vicon/body', 'vicon/marker')
```
**For ROS:**

```python
from SimpleHandEye.interfaces.utils import addNoeticPath
addNoeticPath('/opt/ros/noetic')
from SimpleHandEye.interfaces.ros import ROSTFInterface, initRosNode

initRosNode()
marker_pose_listener = ROSTFInterface('vicon/body', 'vicon/marker')
```

Test the interface and maker sure you can read the pose from Vicon:

```python
marker_pose_listener.getPose()
```
#### Tracking $\mathbf{{}^{cam1}T_{tag}}$ :

In this example, we use a Realsense camera so first we need to instantiate our Realsense camera wrapper class to read images and camera parameters:

```python
from SimpleHandEye.interfaces.cameras import RealSenseCamera
import cv2

def showImage(color_frame, depth_frame, ir1_frame, ir2_frame):
    cv2.imshow('image', color_frame)
    cv2.waitKey(33)

camera = RealSenseCamera(callback_fn=showImage)

intrinsics_params = camera.getIntrinsics()
K = intrinsics_params['RGB']['K']
D = intrinsics_params['RGB']['D']
```

After running above, a new window pops up with a live stream from the camera. We can access to the latest images through:

```python
img = camera.color_frame
```
**Note**: In case the image was available in the form of ROS messages, we could have used our ROS2/ROS image listener classes.

Finally, to track the pose of the tag, we can use our Apriltag tracker class. We could also listen to TF messages published by any kind of third-party trackers through ROS. 

```python
from SimpleHandEye.interfaces.apriltag import ApriltagTracker

tag_pose_tracker = ApriltagTracker(tag_size=0.172, # put your tag size here
                          intrinsic_matrix=K,
                          distortion_coeffs=D)
```

We can query the pose of a tag with arbitrary ID as simply by giving the image from the camera and the requested ID to the `getPose` method of the tracker:

```python
tag_pose_tracker.getPose(camera.color_frame, tag_id=0)
```
#### Formulating the Problem and Collecting Data

