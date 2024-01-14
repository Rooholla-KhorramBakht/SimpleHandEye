import symforce
symforce.set_epsilon_to_symbol()
import symforce.symbolic as sf 
import numpy as np
import pickle 
from symforce.opt.optimizer import Optimizer
from symforce.values import Values
from symforce.opt.factor import Factor
import time

class AXYBReprojectionOptimizer():
    def __init__(self, camera_T_tag = 'A', camera_matrix = None, distortion_coeffs= None):
        """
        A class to optimize the reprojection error of the AXYB calibration method
        @param camera_T_tag: which transformation represents the camera pose tracker, 'A' or 'B' or 'A_inv' or 'B_inv'
        @param camera_matrix: the camera intrinsic matrix
        @param distortion_coeffs: the camera distortion coefficients
        """
        assert camera_T_tag in ['A', 'B', 'A_inv', 'B_inv'], "camera_T_tag must be 'A' or 'B' or 'A_inv' or 'B_inv'"
        self.camera_T_tag = camera_T_tag
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = distortion_coeffs
        self.result = None

    def reprojectionResidual(self_obj,
                            A: sf.Pose3,
                            X: sf.Pose3,
                            Y: sf.Pose3,
                            B: sf.Pose3,
                            tag_p: sf.V3,
                            pix_p: sf.V2,
                            K: sf.Matrix33,
                            sigma: sf.Scalar,
                            epsilon:sf.Scalar):
        """
        The reprojection residual function
        @param A: the pose A in the AX=YB equation
        @param X: the pose X in the AX=YB equation
        @param Y: the pose Y in the AX=YB equation
        @param B: the pose B in the AX=YB equation
        @param tag_ps: the tag position in the tag frame for each image frame
        @param pix_ps: the measured image points for each image frame
        @param K: the camera intrinsic matrix
        @param sigma: the measurement uncertainty
        @param epsilon: a small value to avoid division by zero
        @return: the reprojection residual
        """
        
        if self_obj.camera_T_tag == 'A':
            camera_T_tag = X.inverse()*Y*B
        elif self_obj.camera_T_tag == 'B':
            camera_T_tag = Y.inverse()*A*X
        elif self_obj.camera_T_tag == 'A_inv':
            camera_T_tag = (X.inverse()*Y*B).inverse()
        elif self_obj.camera_T_tag == 'B_inv':
            camera_T_tag = (Y.inverse()*A*X).inverse()
        
        z_hat = K*(camera_T_tag*sf.V3(tag_p))
        z_hat = z_hat/(z_hat[2]+epsilon)
        return sf.V1((z_hat[:2]-pix_p).norm(epsilon=epsilon))/sigma

    def solve(self, A: list, 
                    X: np.array, 
                    Y: np.array, 
                    B: list, 
                    tag_ps: list, 
                    pix_ps: list):
        """
        Solve the AX=YB equation
        @param A: The list of A 4x4 poses in the AX=YB equation
        @param X: the 4x4 pose X in the AX=YB equation
        @param Y: the 4x4 pose Y in the AX=YB equation
        @param B: The list of 4x4 B poses in the AX=YB equation
        @param tag_ps: the tag position in the tag frame (nxm where n is the number of poses and m is the number of correspondences per pose)
        @param pix_ps: the measured image points (nxm where n is the number of poses and m is the number of correspondences per pose)
        @return: the optimized X and Y poses
        """
        assert len(A) == len(B), "Measurement lists must have the same length"
        assert len(A) > 0, "All lists must have at least one element"
        assert self.camera_matrix is not None, "camera_matrix must be set"

        initial_values = Values(
                                A = [sf.Pose3(R = sf.Rot3.from_rotation_matrix(T[0:3,0:3]),
                                                                t = sf.V3(T[0:3,-1]))\
                                                                for T in A ],

                                B = [sf.Pose3(R = sf.Rot3.from_rotation_matrix(T[0:3,0:3]),
                                                                t = sf.V3(T[0:3,-1]))\
                                                                for T in B ],
                                
                                Y = sf.Pose3(R = sf.Rot3.from_rotation_matrix(Y[0:3,0:3]),
                                             t = sf.V3(Y[0:3,-1])),

                                X = sf.Pose3(R = sf.Rot3.from_rotation_matrix(X[0:3,0:3]),
                                                        t = sf.V3(X[0:3,-1])),
                                K = sf.Matrix33(self.camera_matrix),
                                pix_ps = [[sf.V2(pix) for pix in pixels] for pixels in pix_ps],
                                tag_ps = [[sf.V3(c) for c in corners] for corners in tag_ps],
                                sigma = sf.Scalar(float(len(pix_ps))),
                                epsilon = sf.numeric_epsilon,
                                )  
        self.initial_values = initial_values

        factors = []
        i=0
        for i in range(len(pix_ps)):
            for j in range(len(pix_ps[i])):
                factors.append(
                            Factor(
                                    residual=self.reprojectionResidual,
                                    keys=[ 
                                        f"A[{i}]",
                                        f"X",
                                        f"Y",
                                        f"B[{i}]",
                                        f'tag_ps[{i}][{j}]',
                                        f'pix_ps[{i}][{j}]',
                                        "K",
                                        "sigma",
                                        "epsilon"],
                                )
                            ) 
        if self.camera_T_tag in ['B', 'B_inv']:
            optimizer = Optimizer(
                factors=factors,
                optimized_keys=["X", "Y"]+ \
                               [f'B[{i}]' for i in range(len(B))],
                # So that we save more information about each iteration, to visualize later:
                debug_stats=True,
                params=Optimizer.Params(verbose=True, initial_lambda=1e3, lambda_down_factor=1 / 10.0, lambda_upper_bound=1e8, iterations=1000, early_exit_min_reduction=1e-4)
            )
        else:
            optimizer = Optimizer(
                factors=factors,
                optimized_keys=["X", "Y"] + \
                            [f'A[{i}]' for i in range(len(A))],
                # So that we save more information about each iteration, to visualize later:
                debug_stats=False,
                params=Optimizer.Params(verbose=True, initial_lambda=1e3, lambda_down_factor=1 / 10.0, lambda_upper_bound=1e8, iterations=1000, early_exit_min_reduction=1e-4)
            )
        result = optimizer.optimize(initial_values)
        time.sleep(0.1)
        if result.status == Optimizer.Status.SUCCESS:
            print("Optimization successful!")
            self.result = result
        else:
            print("Optimization failed!")
            self.result = None

        return result
    

    def getOptimizedResults(self):
        if self.result is None:
            print("No optimization has been performed yet or optimization failed")
            return None
        X_R = self.result.optimized_values['X'].R.to_rotation_matrix()
        X_t = self.result.optimized_values['X'].t

        Y_R = self.result.optimized_values['Y'].R.to_rotation_matrix()
        Y_t = self.result.optimized_values['Y'].t

        X = np.vstack([np.hstack([X_R, X_t.reshape(3,1)]), np.array([[0,0,0,1]])])
        Y = np.vstack([np.hstack([Y_R, Y_t.reshape(3,1)]), np.array([[0,0,0,1]])])
        return {'X': X, 'Y': Y}