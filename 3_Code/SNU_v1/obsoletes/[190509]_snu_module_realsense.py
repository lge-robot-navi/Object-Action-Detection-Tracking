"""
SNU Detection and MOT Module v1.5 (Realsense [d-435] adapted version)

    Detection Module
        - Code written/modified by : [XYZ] (xyz@qwerty.ac.kr)

    Tracking Module
        - Code written/modified by : [Kyuewang Lee] (kyuewang5056@gmail.com)

    Code Environment
        - python 2.7
        - tensorflow == 1.5.0
        - CUDA 9.0
            -> with cuDNN 7.0.5
        - ROS-kinetics

        < Dependencies >
            - [scikit-learn], [scikit-image], [FilterPy]
            - [numpy], [numba], [scipy], [matplotlib], [opencv-python]

    Source Code all rights reserved
"""

# Import Modules
import os
import cv2
import datetime
import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Import SNU Modules
import src.snu_det_mot_module.scripts.detector__lighthead_rcnn as detector
import src.snu_det_mot_module.scripts.mot__multimodal as mmodaltracker
import src.snu_det_mot_module.scripts.mot_module as mot


# Parameter Struct
class STRUCT:
    def __init__(self):
        pass


################# Realsense Options #################
is_realsense = True

DEPTH_CLIPPED_VALUE = -1
DEPTH_CLIP_DISTANCE = 10    # (in meters)
#####################################################

################## SNU Module Options ##################
###### Detection Parameters ######
model_base_path = os.path.dirname(os.path.abspath(__file__))
detector_name = "lighthead_rcnn"
model_path = model_base_path + "/" + detector_name + "/model/detector.ckpt"
gpu_id = '0'

# Push in to the parameter struct
detparams = STRUCT
detparams.model_path = model_path
detparams.gpu_id = gpu_id
##################################

######### MOT Parameters #########
# [Tracklet Candidate --> Tracklet] Association Age
trkc_to_trk_asso_age = 5

# Destroy Objects
unasso_trk_destroy = 5          # Destroy unassociated tracklets with this amount of continuous unassociation
unasso_trkc_destroy = trkc_to_trk_asso_age + 1      # Destroy unassociated tracklet candidates ""

# Association Threshold
cost_thresh = 0.5

# Depth Histogram Bin Number
dhist_bin = 100

# Push in to the parameter struct
motparams = STRUCT
motparams.unasso_trk_destroy = unasso_trk_destroy
motparams.unasso_trkc_destroy = unasso_trkc_destroy
motparams.cost_thresh = cost_thresh
motparams.trkc_to_trk_asso_age = trkc_to_trk_asso_age
motparams.dhist_bin = dhist_bin
motparams.DEPTH_CLIP_DIST = DEPTH_CLIP_DISTANCE
##################################

###### Visualization Options ######
is_vis_detection = False
is_vis_tracking = False

# Save Figure Options
is_save_fig = False

# openCV Font Options
CV_FONT = cv2.FONT_HERSHEY_PLAIN
###################################
########################################################


# Detector Function
def snu_detector(image, infer_func, inputs):
    # Start Timestamp for DETECTION
    DET_TS_START = datetime.datetime.now()
    # Activate Detection Module
    result_dict = detector.detector(image, infer_func, inputs)
    # Convert to Detection BBOXES
    curr_dets = detector.get_detection_bboxes(result_dict)
    # Stop Timestamp for DETECTION
    DET_TS_STOP = datetime.datetime.now()
    # Elapsed Time for the DETECTION MODULE (ms)
    DET_ELAPSED_TIME = (DET_TS_STOP - DET_TS_START).total_seconds() * 1000

    return curr_dets, DET_ELAPSED_TIME


# MMMOT Function
def snu_mmmot(color_image, depth_image, fidx, dets, motparams, trackers, tracker_cands):
    # Start Timestamp for MultiModal Multi-Object Tracker
    MMMOT_TS_START = datetime.datetime.now()
    # MMMOT Module
    trackers, tracker_cands = mmodaltracker.tracker(color_image, depth_image, fidx, dets, motparams, trackers, tracker_cands)
    # STOP Timestamp for MultiModal Multi-Object Tracker
    MMMOT_TS_STOP = datetime.datetime.now()
    # Elapsed Time for the MMMOT Module (ms)
    MMMOT_ELAPSED_TIME = (MMMOT_TS_STOP - MMMOT_TS_START).total_seconds() * 1000

    return trackers, tracker_cands, MMMOT_ELAPSED_TIME


# Visualize Detections
def visualize_detections(dets, fig_axes):
    for det in dets:
        det = det.astype(np.int32)
        det_bbox = patches.Rectangle((det[0], det[1]),
                                     det[2] - det[0], det[3] - det[1],
                                     fill=False,
                                     lw=3,
                                     ec=[1, 0, 0])
        fig_axes.add_patch(det_bbox)
        fig_axes.set_adjustable('box-forced')
    return fig_axes


# Visualize Tracklets
def visualize_tracklets(trks, colors, fig_axes):
    for trk in trks:
        zs, _ = mot.zx_to_bbox(trk.states[-1])
        zs = zs.astype(np.int32)
        state_bbox = patches.Rectangle((zs[0], zs[1]),
                                       zs[2]-zs[0], zs[3]-zs[1],
                                       fill=False,
                                       lw=3,
                                       ec=colors[(trk.id % 3), :])
        fig_axes.add_patch(state_bbox)
        fig_axes.set_adjustable('box-forced')
    return fig_axes


# Visualize Detections (openCV version)
def visualize_detections_2(img, dets, line_width=2):
    for det in dets:
        det = det.astype(np.int32)

        # Draw Rectangle BBOX
        cv2.rectangle(img, (det[0], det[1]), (det[2], det[3]),
                      (255, 0, 0), line_width)


# Visualize Tracklets (openCV version)
def visualize_tracklets_2(img, trks, colors, line_width=2):
    for trk in trks:
        zs, _ = mot.zx_to_bbox(trk.states[-1])
        zs = zs.astype(np.int32)

        # Tracklet BBOX Color
        trk_color = colors[(trk.id % 3), :] * 255

        # Draw Rectangle BBOX
        cv2.rectangle(img, (zs[0], zs[1]), (zs[2], zs[3]),
                      (trk_color[0], trk_color[1], trk_color[2]), line_width)


# Main Function
def main():
    # Load Detection Model
    infer_func, inputs = detector.load_model(model_path, gpu_id)

    # Visualization Figure
    # (will be changed to openCV figure later)
    if (is_vis_detection is True) or (is_vis_tracking is True):
        plt.ion()
        fig = plt.figure()

    # Tracklet Color (Visualization)
    # (Later, generate colormap)
    trk_colors = np.random.rand(32, 3)

    if is_realsense is True:
        # Configure Depth and Color Streams
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start Streaming
        profile = pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", depth_scale)

        clipping_distance_in_meters = DEPTH_CLIP_DISTANCE  # 1 meter
        clipping_distance = clipping_distance_in_meters / depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)

        # Initialize Tracklet and Tracklet Candidate Object
        trackers = []
        tracker_cands = []

        # Initialize Frame Index
        fidx = 0

        try:
            while True:
                # Increase Frame Index
                fidx += 1

                # Wait for a coherent pair of frames: RBGD
                frames = pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = align.process(frames)

                # DEPTH (Aligned depth frame)
                depth_frame = aligned_frames.get_depth_frame()

                # RGB
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # Convert Images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Clip Depth Image
                clipped_depth_image = np.where((depth_image > clipping_distance) | (depth_image <= 0), DEPTH_CLIPPED_VALUE, depth_image)

                # Apply ColorMap on DEPTH Image
                # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

                # DETECTION MODULE
                curr_dets, DET_TIME = snu_detector(color_image, infer_func, inputs)

                # MultiModal Tracking MODULE
                trackers, tracker_cands, MMMOT_TIME = snu_mmmot(color_image, clipped_depth_image, fidx, curr_dets, motparams, trackers, tracker_cands)

                # Detection Visualization
                if is_vis_detection is True:
                    visualize_detections_2(color_image, curr_dets, line_width=2)

                # MMMOT Visualization
                if is_vis_tracking is True:
                    visualize_tracklets_2(color_image, trackers, trk_colors, line_width=3)

                # Detection Speed Visualization
                det_fps = "Detector Speed: " + str(round(1000/DET_TIME, 2)) + " (fps)"
                mmmot_fps = "Tracker Speed: " + str(round(1000/MMMOT_TIME, 2)) + " (fps)"
                cv2.putText(color_image, det_fps, (10, 20), CV_FONT, 1.3, (255, 0, 255), thickness=2)
                cv2.putText(color_image, mmmot_fps, (10, 50), CV_FONT, 1.3, (255, 0, 255), thickness=2)

                # Visualization Window (using OpenCV-Python)
                cv2.imshow('Tracking', color_image)

                # OpenCV imshow window
                cv2.waitKey(1)
        finally:
            # Stop Streaming
            pipeline.stop()

            # Destroy All opencv windows
            cv2.destroyAllWindows()

    else:
        # Sequence Path
        seq_basepath = '../DATA/'
        sequences = ['ETH-Crossing', 'KITTI-17']

        # For every image sequences
        for seq_num, seq in enumerate(sequences):
            # Sequence Number
            seq_num += 1

            # Current Sequence Path
            seqpath = seq_basepath + seq

            # Current Image Sequence Paths
            seq_imgspath = seqpath + '/imgs/'

            # Load Image Sequence Lists
            imgNameList = os.listdir(seq_imgspath)
            imgList = [seq_imgspath + filename for filename in imgNameList]
            imgList.sort()

            # Generalize Save Path

            # Initialize Tracklet and Tracklet Candidate Object
            trackers = []
            tracker_cands = []

            # For every frames in the image sequence
            for fidx, frameName in enumerate(imgList):
                # Frame Index Starts from 1
                fidx += 1

                # Load Current Frame Image
                # frame = io.imread(frameName)
                frame = cv2.imread(frameName)

                # Visualization Window
                if (is_vis_detection is True) or (is_vis_tracking is True):
                    ax1 = fig.add_subplot(111, aspect='equal')
                    ax1.imshow(frame)
                    plt.title('Sequence [' + seq + ']')

                # DETECTION MODULE
                curr_dets, DET_TIME = snu_detector(frame, infer_func, inputs)

                # MultiModal Tracking MODULE
                trackers, tracker_cands, MMMOT_TIME = snu_mmmot(frame, fidx, curr_dets, motparams, trackers, tracker_cands)

                # Detection Visualization
                if is_vis_detection is True:
                    ax1 = visualize_detections(curr_dets, ax1)

                # MMMOT Visualization
                if is_vis_tracking is True:
                    ax1 = visualize_tracklets(trackers, trk_colors, ax1)

                # Draw Current Frame Result
                if (is_vis_detection is True) or (is_vis_tracking is True):
                    fig.canvas.flush_events()
                    plt.draw()
                    if ax1 is not None:
                        ax1.cla()


# Main Function
if __name__ == '__main__':
    main()
