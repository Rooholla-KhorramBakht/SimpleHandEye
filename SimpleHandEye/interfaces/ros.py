import threading
import numpy as np
import rospy
import tf
import tf.transformations as transformations
from SimpleHandEye.interfaces.base import BasePoseInterface
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

def terminateRosNode():
    """
    Terminates the ROS node within the current python process.
    """
    rospy.signal_shutdown("Node closed by user")


def initRosNode(node_name="simplehandeye_node"):
    """
    Initialize a ROS node within the current python process.

    Parameters:
        node_name (str, optional): The name of the ROS node. Defaults to 'b1py_node'.
    """
    rospy.init_node(node_name)
    rospy.loginfo("Node initialized")

class ROSCameraReader:
    """
    A class for interacting with ROS1 camera topics.

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
        rospy.init_node('ros1_camera_reader', anonymous=True)
        self.image_topic = image_topic
        self.camera_info_topic = camera_info_topic
        self.bridge = CvBridge()
        self.color_frame = None
        self.camera_info = None
        self.K = K
        self.D = D

        # Subscribers
        self.image_subscriber = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        if self.camera_info_topic:
            self.camera_info_subscriber = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        
        # Start the thread for listening to topics
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self.spin)
        self._thread.start()

    def spin(self):
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            rospy.spin()

    def image_callback(self, data):
        """
        Callback function for the image topic.
        """
        try:
            self.color_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def camera_info_callback(self, data):
        """
        Callback function for the camera info topic.
        """
        self.camera_info = data
        self.K = np.array(data.K).reshape((3, 3))
        self.D = np.array(data.D)

    def close(self):
        """
        Closes the ROS1CameraReader object, stopping the subscriber and the thread.
        """
        self._stop_event.set()
        self._thread.join()
        rospy.signal_shutdown('Closing ROS1CameraReader')

class ROSTFInterface(BasePoseInterface):
    """
    This class listens to ROS TF messages and keeps track of the transformation
    between a parent and child frame as they get published.
    The pose is represented as a 4x4 homogeneous transformation matrix.

    Attributes:
        parent_frame (str): The name of the parent frame.
        child_frame (str): The name of the child frame.
        rate (int): Rate at which the TF listener updates.
        callback (func): A user-defined callback function that takes a 4x4
                         homogeneous matrix as input.
        T (ndarray): 4x4 pose matrix of child with respect to parent.
    """

    def __init__(self, parent_frame, child_frame, callback=None, rate=100):
        """
        Initialize the TF listener.

        Parameters:
            parent_frame (str): The name of the parent frame.
            child_frame (str): The name of the child frame.
            callback (func, optional): User-defined callback function.
            rate (int, optional): Rate of TF update. Defaults to 100.
        """
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.rate = rate
        self.callback = callback
        super().__init__()

        # Initialize the TF listener
        self.tf_listener = tf.TransformListener()
        self.stop_thread = False
        self.thread = threading.Thread(target=self.update)
        self.thread.start()

    def getPose(self):
        if self.T is not None:
            return self.T
        else:
            return None
    
    def update(self):
        """
        Update the transformation matrix. This function runs in a separate thread
        and continuously listens for TF messages to update the transformation matrix.
        """
        rate = rospy.Rate(self.rate)
        while not self.stop_thread and not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    self.parent_frame, self.child_frame, rospy.Time(0)
                )
                if trans is not None and rot is not None:
                    # Update the rotation part of the homogeneous transformation matrix
                    self.T = np.eye(4)
                    self.T[:3, :3] = transformations.quaternion_matrix(rot)[:3, :3]

                    # Update the translation part of the homogeneous transformation matrix
                    self.T[:3, 3] = trans
                else:
                    self.T = None

                # If a callback is provided, call it
                if self.callback:
                    self.callback(self.T)

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
            ):
                rospy.loginfo("TF not ready")

            rate.sleep()

    def close(self):
        """
        Stops the thread that is updating the transformation matrix.
        """
        self.stop_thread = True
        self.thread.join()


class ROSTFPublisher:
    """
    This class allows users to publish ROS TF messages to describe the transformation
    between a parent and child frame. The pose is represented as a 4x4 homogeneous
    transformation matrix.

    Attributes:
        parent_frame (str): The name of the parent frame.
        child_frame (str): The name of the child frame.
        broadcaster (tf.TransformBroadcaster): The TF broadcaster used for publishing TFs.
    """

    def __init__(self, parent_frame, child_frame):
        """
        Initialize the TF publisher.

        Parameters:
            parent_frame (str): The name of the parent frame.
            child_frame (str): The name of the child frame.
        """
        self.parent_frame = parent_frame
        self.child_frame = child_frame

        # Initialize the TF broadcaster
        self.broadcaster = tf.TransformBroadcaster()

    def publish(self, T):
        """
        Publishes the transformation matrix as a TF message.

        Parameters:
            T (ndarray): 4x4 homogeneous transformation matrix.
        """
        # Extract translation
        trans = T[:3, 3]

        # Extract rotation and convert to quaternion
        rot = transformations.quaternion_from_matrix(T)

        # Publish the TF
        self.broadcaster.sendTransform(
            trans, rot, rospy.Time.now(), self.child_frame, self.parent_frame
        )