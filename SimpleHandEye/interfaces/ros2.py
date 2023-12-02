from SimpleHandEye.interfaces.base import BasePoseInterface
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading

class ROS2TFInterface(BasePoseInterface, Node):
    def __init__(self, parent_name, child_name, node_name='tf2_listener'):
        Node.__init__(self, node_name)
        BasePoseInterface.__init__(self)
        self.node_name = node_name
        self.parent_name = parent_name
        self.child_name = child_name
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.running = True 
        self.thread = threading.Thread(target=self.run).start() 


    def run(self): 
        while rclpy.ok() and self.running: 
            rclpy.spin_once(self)
            try:
                trans = self.tfBuffer.lookup_transform(self.parent_name, self.child_name, rclpy.time.Time())
                # Extract translation and rotation from the transform
                translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
                rotation = [trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z]
                # Update the rotation part of the homogeneous transformation matrix
                self.T = np.eye(4)
                self.T[0:3, 0:3]=R.from_quat(rotation).as_matrix()
                self.T[:3, 3] = translation
                self.stamp = trans.header.stamp.nanosec*1e-9 + trans.header.stamp.sec
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error('Error in lookupTransform: %s' % e)
                self.T = None
                self.stamp = None

    def getPose(self):
        if self.T is None:
            return None
        else:
            return self.T, self.stamp
        
    def close(self):
        self.running = False
        self.thread.join()
        self.destroy_node()