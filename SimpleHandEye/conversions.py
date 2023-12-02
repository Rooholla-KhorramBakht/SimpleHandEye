from scipy.spatial.transform import Rotation as R
import numpy as np

def q2R(q):
    '''
    Convert a quaternion to rotation matrix 
    input: q: quaternion in the form of [w, x, y, z]
    '''
    return R.from_quat(q).as_matrix()

def R2q(R):
    '''
    Convert a rotation matrix to quaternion
    input: R: 3x3 rotation matrix
    '''
    return R.from_matrix(R).as_quat()

def q2vec(q):
    '''
    Convert a quaternion to a 3D vector
    '''
    return R.from_quat(q).as_rotvec()

def vec2q(vec):
    '''
    Convert a 3D vector to a quaternion
    '''
    return R.from_rotvec(vec).as_quat()

def qt2T(q, t):
    '''
    Convert a quaternion and translation vector to a homogeneous transformation matrix
    '''
    T = np.eye(4)
    T[0:3, 0:3]=q2R(q)
    T[:3, 3] = t
    return T

def T2qt(T):
    '''
    Convert a homogeneous transformation matrix to a quaternion and translation vector
    '''
    q = R2q(T[0:3, 0:3])
    t = T[:3, 3]
    return q, t

