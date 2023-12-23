from SimpleHandEye.interfaces.base import BasePoseInterface
import numpy as np
import cv2
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

class ChessboardTracker(BasePoseInterface):
    """
    A class for tracking the pose of a chessboard calibration target in images.

    Attributes:
        intrinsic_matrix (numpy.ndarray): The camera's intrinsic matrix.
        distortion_coeffs (numpy.ndarray): The camera's distortion coefficients.
        board_size (tuple): The size of the chessboard as (columns, rows).
        square_size (float): The size of one square on the chessboard in meters.
        detected_points (list): A list of detected corner points in the chessboard.
        object_points (list): The 3D points in real world space.

    Methods:
        process(self, image):
            Process an image to detect the chessboard and estimate its pose.

        getPose(self, img):
            Get the pose of the chessboard in the image.

        undistortImage(self, img):
            Undistort an input image using camera calibration parameters.
    """

    def __init__(self, board_size, square_size, intrinsic_matrix=None, distortion_coeffs=None):
        """
        Initialize the ChessboardTracker.

        Args:
            board_size (tuple): The size of the chessboard as (columns, rows).
            square_size (float): The size of one square on the chessboard in meters.
            intrinsic_matrix (numpy.ndarray): The camera's intrinsic matrix.
            distortion_coeffs (numpy.ndarray): The camera's distortion coefficients.
        """
        super().__init__()
        self.intrinsic_matrix = intrinsic_matrix
        self.distortion_coeffs = distortion_coeffs
        self.board_size = board_size
        self.square_size = square_size
        self.detected_points = []
        self.object_points = self._createObjectPoints()

    def _createObjectPoints(self):
        """
        Create an array of points representing the corners of the chessboard in real world space.

        Returns:
            numpy.ndarray: The array of 3D points.
        """
        objp = np.zeros((self.board_size[0]*self.board_size[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:self.board_size[0], 0:self.board_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        return objp

    def process(self, image):
        """
        Process an image to detect the chessboard and estimate its pose.

        Args:
            image (numpy.ndarray): The input image to be processed.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.board_size, None)

        if ret:
            if self.distortion_coeffs is not None:
                corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            self.detected_points = corners

    def getPose(self, img):
        """
        Get the pose of the chessboard in the image.

        Args:
            img (numpy.ndarray): The input image to be processed.

        Returns:
            numpy.ndarray: The pose of the chessboard and None if not found.
        """
        if self.distortion_coeffs is not None:
            img = self.undistortImage(img)
        self.process(img)
        if len(self.detected_points) > 0:
            retval, rvec, tvec = cv2.solvePnP(self.object_points, self.detected_points,
                                              self.intrinsic_matrix, self.distortion_coeffs)
            return rvec, tvec
        else:
            return None

    def undistortImage(self, img):
        """
        Undistorts an input image using camera calibration parameters.

        Args:
            img (numpy.ndarray): The input image to be undistorted.

        Returns:
            numpy.ndarray: The undistorted image.
        """
        undistorted_image = cv2.undistort(img, self.intrinsic_matrix, self.distortion_coeffs)
        return undistorted_image


