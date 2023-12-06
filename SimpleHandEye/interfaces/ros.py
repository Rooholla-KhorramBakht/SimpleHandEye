import threading
import numpy as np
import rospy
import tf
import tf.transformations as transformations
from SimpleHandEye.interfaces.base import BasePoseInterface


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