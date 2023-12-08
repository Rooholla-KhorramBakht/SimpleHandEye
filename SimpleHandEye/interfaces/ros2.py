from SimpleHandEye.interfaces.base import BasePoseInterface
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

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

    def __init__(self, image_topic, camera_info_topic=None, K=None, D=None):
        super().__init__('ros2_camera_reader')
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic
        self.bridge = CvBridge()
        self.color_frame = None
        self.camera_info = None
        self.K = K
        self.D = D

        # Subscribers
        self.image_subscriber = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        if self.camera_info_topic:
            self.camera_info_subscriber = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        
        # Start the thread for listening to topics
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self.spin)
        self._thread.start()

    def spin(self):
        while not self._stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(self)

    def image_callback(self, msg):
        """
        Callback function for the image topic.
        """
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        """
        Callback function for the camera info topic.
        """
        self.camera_info = msg
        self.K = np.array(msg.K).reshape((3, 3))
        self.D = np.array(msg.D)

    def close(self):
        """
        Closes the ROS2CameraReader object, stopping the subscriber and the thread.
        """
        self._stop_event.set()
        self._thread.join()
        self.destroy_subscription(self.image_subscriber)
        if self.camera_info_topic:
            self.destroy_subscription(self.camera_info_subscriber)
        self.destroy_node()
        

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
            return self.T
        
    def close(self):
        self.running = False
        self.thread.join()
        self.destroy_node()