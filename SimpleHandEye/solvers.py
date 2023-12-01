import opencv as cv 
import numpy as np

class OpenCVSolver:
    def __init__(self, type: str='AX=YB', method=None):
        assert type in ['AX=YB', 'AX=XB'], "type must be 'AX=YB' or 'AX=XB'"
        if type == 'AX=YB':
            assert method in [cv.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
                              cv.CALIB_ROBOT_WORLD_HAND_EYE_LI,
                              None], "method must be None, cv.CALIB_ROBOT_WORLD_HAND_EYE_SHAH, or cv.CALIB_ROBOT_WORLD_HAND_EYE_LI"
        elif type == 'AX=XB':
            assert method in [cv.CALIB_HAND_EYE_TSAI,
                              cv.CALIB_HAND_EYE_HORAUD,
                              cv.CALIB_HAND_EYE_PARK,
                              cv.CALIB_HAND_EYE_ANDREFF,
                              cv.CALIB_HAND_EYE_DANIILIDIS,
                              None], "method must be None, cv.CALIB_HAND_EYE_TSAI, cv.CALIB_HAND_EYE_HORAUD, cv.CALIB_HAND_EYE_PARK, cv.CALIB_HAND_EYE_ANDREFF, or cv.CALIB_HAND_EYE_DANIILIDIS"
        self.type = type
        self.method = method
        self.solve = {'AX=YB':self.solveAXYB,
                       'AX=XB':self.solveAXXB
                     }[type]

    def solveAXYB(self, A:list, B: list):
        '''
        A: list of 4x4 transformation matrices
        B: list of 4x4 transformation matrices
        return: 4x4 solution transformation matrices X and Y
        '''
        assert len(A) == len(B), "A and B must have the same length"
        assert len(A) >= 3, "A and B must have at least 3 elements"
        A_rot_vecs = [cv.Rodrigues(A[i][:3,:3])[0] for i in range(len(A))]
        B_rot_vecs = [cv.Rodrigues(B[i][:3,:3])[0] for i in range(len(B))]
        A_trans_vecs = [A[i][:3,3] for i in range(len(A))]
        B_trans_vecs = [B[i][:3,3] for i in range(len(B))]
        if self.method is None:
            X_R, X_t, Y_R, Y_t = cv.calibrateRobotWorldHandEye(A_rot_vecs, A_trans_vecs,
                                                            B_rot_vecs, B_trans_vecs)
        else:
            X_R, X_t, Y_R, Y_t = cv.calibrateRobotWorldHandEye(A_rot_vecs, A_trans_vecs,
                                                            B_rot_vecs, B_trans_vecs, method=self.method)
        X = np.eye(4)
        Y = np.eye(4)
        X[:3,:3] = cv.Rodrigues(X_R)[0]
        X[:3,3] = X_t
        Y[:3,:3] = cv.Rodrigues(Y_R)[0]
        Y[:3,3] = Y_t
        return X, Y
    
    def solveAXXB(self, base_T_gripper:list, cam_T_target: list):
        '''
        A: list of 4x4 transformation matrices
        B: list of 4x4 transformation matrices
        return: 4x4 solution transformation matrix X
        '''
        assert len(A) == len(B), "A and B must have the same length"
        assert len(A) >= 3, "A and B must have at least 3 elements"
        base_R_gripper = [cv.Rodrigues(base_T_gripper[i][:3,:3])[0] for i in range(len(base_T_gripper))]
        cam_R_target = [cv.Rodrigues(cam_T_target[i][:3,:3])[0] for i in range(len(base_T_gripper))]
        base_t_gripper = [base_T_gripper[i][:3,3] for i in range(len(base_T_gripper))]
        cam_t_target = [cam_T_target[i][:3,3] for i in range(len(base_T_gripper))]
        if self.method is None:
            gripper_R_cam, gripper_t_cam = cv.calibrateHandEye(base_R_gripper, base_t_gripper,
                                                    cam_R_target, cam_t_target)
        else:
            gripper_R_cam, gripper_t_cam = cv.calibrateHandEye(base_R_gripper, base_t_gripper,
                                                    cam_R_target, cam_t_target, method=self.method)
        
        gripper_T_cam = np.eye(4)
        gripper_T_cam[:3,:3] = cv.Rodrigues(gripper_R_cam)[0]
        gripper_T_cam[:3,3] = gripper_t_cam
        return gripper_T_cam