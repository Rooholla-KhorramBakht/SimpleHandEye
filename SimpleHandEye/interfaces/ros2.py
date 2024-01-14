from SimpleHandEye.interfaces.base import BasePoseInterface
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
import cv2
import time

class ROS2CameraReader(Node):
    """
    A class for interacting with a ROS2 camera topics.

    Args:
        image_topic (str): The topic name for the image stream.
        camera_info_topic (str, optional): The topic name for the camera information.
        K (numpy.ndarray, optional): The intrinsic camera matrix for the raw (distorted) images.
        D (numpy.ndarray, optional): The distortion coefficients.

    Attributes:
        color_frame (numpy.ndarray): The latest image frame received from the topic.
        camera_info (CameraInfo): The latest camera information received from the topic.
        K (numpy.ndarray): The intrinsic camera matrix.
        D (numpy.ndarray): The distortion coefficients.
    """
    def __init__(self, image_topic, node_name, camera_info_topic=None, K=None, D=None):
        super().__init__(f'{node_name}_camera_reader')
        self.bridge = CvBridge()
        self.color_frame = None
        self.camera_info = None
        self.K = K
        self.D = D
        self.node_name = node_name

        self.image_subscriber = self.create_subscription(Image, image_topic, self.image_callback, 10)
        if camera_info_topic:
            self.camera_info_subscriber = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

    def image_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        self.camera_info = msg
        self.K = np.array(msg.k).reshape((3, 3))
        self.D = np.array(msg.d)

    def get_image(self):
        """
        Returns the latest image frame received from the topic.

        Returns:
            numpy.ndarray: The latest image frame received from the topic.
        """
        return self.color_frame 

    def get_intrinsics(self):
        """
        Returns the intrinsic camera matrix.

        Returns:
            numpy.ndarray: The intrinsic camera matrix.
        """
        return {'K': self.K, 'D': self.D}
    def close(self):
        self.destroy_node()


class ROS2TFInterface(Node):

    def __init__(self, parent_name, child_name, node_name):
        super().__init__(f'{node_name}_tf2_listener')
        self.parent_name = parent_name
        self.child_name = child_name
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.T = None
        self.stamp = None
        self.running = True
        self.thread = threading.Thread(target=self.update_loop)
        self.thread.start()
        self.trans = None

    def update_loop(self):
        while self.running:
            try:
                self.trans = self.tfBuffer.lookup_transform(self.parent_name, self.child_name, rclpy.time.Time(), rclpy.time.Duration(seconds=0.1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                pass
            time.sleep(0.01)    

    def get_pose(self):
        if self.trans is None:
            return None
        else:
            translation = [self.trans.transform.translation.x, self.trans.transform.translation.y, self.trans.transform.translation.z]
            rotation = [self.trans.transform.rotation.x, self.trans.transform.rotation.y, self.trans.transform.rotation.z, self.trans.transform.rotation.w]
            self.T = np.eye(4)
            self.T[0:3, 0:3] = R.from_quat(rotation).as_matrix()
            self.T[:3, 3] = translation
            self.stamp = self.trans.header.stamp.nanosec * 1e-9 + self.trans.header.stamp.sec
            return self.T

    def close(self):
        self.running = False
        self.thread.join()  
        self.destroy_node()

class ROS2ExecutorManager:
    def __init__(self):
        self.executor = MultiThreadedExecutor()
        self.nodes = []
        self.executor_thread = None

    def add_node(self, node: Node):
        """Add a new node to the executor."""
        self.nodes.append(node)
        self.executor.add_node(node)

    def _run_executor(self):
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def start(self):
        """Start spinning the nodes in a separate thread."""
        self.executor_thread = threading.Thread(target=self._run_executor)
        self.executor_thread.start()

    def close(self):
        """Terminate all nodes and shutdown rclpy."""
        for node in self.nodes:
            node.destroy_node()
        # if self.executor_thread:
        #     self.executor_thread.join()