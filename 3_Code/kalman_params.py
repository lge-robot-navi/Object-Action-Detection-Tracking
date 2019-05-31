"""
Kalman Filter Parameters for the MOT Algorithm

Code Written by
==>> Kyuewang Lee
        from Seoul National University, South Korea
        (currently in Perception and Intelligence Laboratory)

        Contact: kyuewang5056@gmail.com

"""

import numpy as np

# Motion Model [x, y, dx, dy, w, h] (6x6)
# in the future, change the overall parameters according to [x, y, z, dx, dy, dz, w, h]
A = np.float32([[1, 0, 1, 0, 0, 0],
                [0, 1, 0, 1, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]])

# Error Covariance Matrix (6x6)
P = np.float32([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]]) * 1e-3
P = np.multiply(P, P)

# Unit Transformation Matrix (6x6)
H = np.float32([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]])

# State Covariance Matrix (6x6)
Q = np.float32([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]]) * 1e-2
Q = np.multiply(Q, Q)

# Measurement Covariance Matrix (6x6)
R = np.float32([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]]) * 1e-3
R = np.multiply(R, R)

# Kalman Gain Initialization (6x6)
K = np.float32([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1]])










