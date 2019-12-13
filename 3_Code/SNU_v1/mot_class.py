"""
SNU MOT Class File
    - Code written by Kyuewang Lee (kyuewang5056@gmail.com)
    - from Seoul National University, South Korea

Baseline Code Link
    - [Link] https://github.com/abewley/sort
    - Bewley et. al. "Simple Online and Realtime Tracking"
    - [2016 IEEE International Conference on Image Processing, ICIP]

* Note that this code merely referenced the coding style of Bewley's work

"""

# Import Libraries
import copy
import numpy as np
import filterpy.kalman.kalman_filter as kalmanfilter
# from filterpy.kalman import KalmanFilter

# Import Source Libraries
import mot_module as mot
import kalman_params as kparams


# Tracklet Candidate Class
class TrackletCandidate(object):
    # Initialization
    def __init__(self, bbox):
        self.age = 1
        self.asso_dets = [bbox]
        self.is_associated = [True]
        self.z = [mot.bbox_to_zx(bbox, np.zeros(2))]


    # Destructor
    def __del__(self):
        print("Tracklet Candidate Destroyed!")

    # Update
    def update(self, bbox=None):
        self.age += 1
        if bbox is None:
            self.asso_dets.append(None)
            self.is_associated.append(False)
            self.z.append(None)
        else:
            z_bbox = mot.bbox_to_zx(bbox)
            velocity = (z_bbox[0:2] - self.z[-1][0:2]).reshape(2)
            self.asso_dets.append(bbox)
            self.is_associated.append(True)
            self.z.append(mot.bbox_to_zx(bbox, velocity))


# Tracklet Class
class Tracklet(object):
    # Initialization
    def __init__(self, bbox, fidx, trk_id):
        # Tracklet ID
        self.id = trk_id
        # Tracklet Age
        self.age = 1
        # Tracklet Birth Frame
        self.birth_fidx = fidx
        # Associated Detections
        self.asso_dets = [bbox]
        # Association Counter
        self.is_associated = [True]

        # Tracklet Kalman Parameter Initialization
        self.A = kparams.A      # State Transition Matrix (Motion Model)
        self.H = kparams.H      # Unit Transformation Matrix
        self.P = kparams.P      # Error Covariance Matrix
        self.Q = kparams.Q      # State Covariance Matrix
        self.R = kparams.R      # Measurement Covariance Matrix

        # self.kf = KalmanFilter(dim_x=6, dim_z=6)
        # self.kf.F = kparams.A  # State Transition Matrix (Motion Model)
        # self.kf.H = kparams.H  # Unit Transformation Matrix
        # self.kf.P = kparams.P  # Error Covariance Matrix
        # self.kf.Q = kparams.Q  # State Covariance Matrix
        # self.kf.R = kparams.R  # Measurement Covariance Matrix

        # Kalman States (Initial States)
        init_state = mot.bbox_to_zx(bbox, np.zeros(2))
        self.x = init_state
        self.xp, self.Pp = kalmanfilter.predict(self.x, self.P, self.A, self.Q)

        # Tracklet States
        self.states = [self.x]

        # Tracklet Prediction States (Next State Prediction)
        self.pred_states = [self.xp]

        # Depth-related Variables
        self.depth_hist = None
        self.depth = None

    # Destructor
    def __del__(self):
        print("Tracklet [id: " + str(self.id) + "] Destroyed!")

    # Tracklet Update
    def update(self, bbox=None):
        # Increase Tracklet Age
        self.age += 1

        # Get Detection (Measurement)
        if bbox is None:
            # If Tracklet is unassociated,
            # replace detection with the previous Kalman prediction
            self.asso_dets.append(None)
            self.is_associated.append(False)
            z = self.pred_states[-1]
            z = np.array([z[0], z[1], np.zeros(1), np.zeros(1), z[4], z[5]])
        else:
            z_bbox = mot.bbox_to_zx(bbox)
            if self.asso_dets[-1] is None:
                prev_z_bbox = self.states[-1]
            else:
                prev_z_bbox = mot.bbox_to_zx(self.asso_dets[-1])
            velocity = (z_bbox[0:2] - prev_z_bbox[0:2]).reshape(2)
            self.asso_dets.append(bbox)
            self.is_associated.append(True)
            z = mot.bbox_to_zx(bbox, velocity)

        # Kalman Update
        # self.kf.update(z)
        self.x, self.P = kalmanfilter.update(self.xp, self.Pp, z, self.R, self.H)

        # Append Tracklet State
        self.states.append(self.x)

    # Tracklet Prediction
    def predict(self):
        # Kalman Prediction
        # self.kf.predict()
        self.xp, self.Pp = kalmanfilter.predict(self.x, self.P, self.A, self.Q)
        self.pred_states.append(self.xp)

    # Get State in the Specific Frame Index
    def get_state(self, fidx):
        states_idx = (fidx - self.birth_fidx)
        return self.states[states_idx]

    # Update Depth Information
    def update_depth(self, depth_hist, clip_distance):
        self.depth_hist = depth_hist

        dhist_bin = len(depth_hist)
        max_bin = depth_hist.argmax() + 1
        self.depth = (max_bin / (dhist_bin * 1.0)) * clip_distance
