#!/usr/bin/env python
## @package pyexample
#  Documentation for this module.
#
#  Copyright GPL-3.0
#
#  More details.
"""
SNU Integrated Module v2.5
    - Main Execution Code (Run Code)

    [Updates on v2.0] (~190908)
    - Code Structure is modified
    - Subscribe multi-modal sensor input in a <struct>-like data structure

    [Updates on v2.05] (~190920)
    - Separated ROS embedded mode and Ordinary run mode (tentative, to-be-updated)
    - Improved <struct>-like multi-modal sensor data structure

    [Updates on v2.5] (~191216)
    - Data Synchronization on 5 multi-modal sensor data
    - TBA

    [Update Expected on v3.0]
    - <class>-like multi-modal sensor data structure
    - separate "static" and "dynamic" agent modes
    - enhanced visualization method (such as CPU multi-threading)
    - (if possible) make rospy Subscriber and Publisher in a <class> method
        : remove global variables
    - TBA

    << Project Information >>
    - "Development of multimodal sensor-based intelligent systems for outdoor surveillance robots"
    - Project Total Period : 2017-04-01 ~ 2021-12-31
    - Current Project Period : The 3rd year (2019-01-01 ~ 2019-12-31)

    << Institutions & Researchers >>
    - Perception and Intelligence Laboratory (PIL)
        [1] Kyuewang Lee (kyuewang@snu.ac.kr)
        [2] Daeho Um (umdaeho1@gmail.com)
    - Machine Intelligence and Pattern Recognition Laboratory (MIPAL)
        [1] Jae-Young Yoo (yoojy31@snu.ac.kr)
        [2] Hojun Lee (hojun815@snu.ac.kr)
        [3] Inseop Chung (jis3613@snu.ac.kr)

    << Code Environments >>
        [ Key Environments ]
        - python == 2.7
        - torch == 1.0.0
            - torchvision == 0.2.0
        - CUDA == 10.0 (greater than 9.0)
            - cuDNN == 7.5.0
        - ROS-kinetics
            - (need rospkg inside the python virtualenv)
        - opencv-python
        - empy
            - (need this to prevent "-j4 -l4" error)

        [ Other Dependencies ] - (will be updated continuously)
        - numpy, numba, scipy, opencv-python, FilterPy, yaml, rospkg, sklearn

    << Memo >>
        Watch out for OpenCV imshow!!!
            -> RGB is shown in "BGR" format
        ** Notes about RGB Image Input
            -> rosbag file from pohang agents : BGR format
            -> rostopic from D435i Camera : RGB format

"""

# Import Modules
import cv2
import socket
import os
import datetime
import snu_utils.general_functions as gfuncs

# Import Data Structure Module
from data_struct import *

# Import ROS Util Functions
from ros_utils import wrap_tracks as ros_publish_wrapper

# Import SNU Algorithm Modules
import module_detection as snu_det
import module_tracking as snu_mmt
import module_action as snu_acl

import snu_utils.bounding_box as fbbox

# Rescue Options
import rescue.force_thermal_align_iitp_final_night as rgb_t_align


#### ROS Embedding Selection ####
is_ros_embedded = False
#################################

### Agent Type and Name ###
agent_type = "dynamic"
agent_name = socket.gethostname()
###########################

# IS DEPTH NOT ALIGNED ??
is_depth_not_aligned = False

### Use SSH? ###
# If SSH Mode is True, all visualization turns off
# snu result image topic is however, published
is_ssh_mode = False

# Test Mode
is_test_code = True

# Save Image Sequences
is_save_img_seq = True
is_save_override = True

# Get Screen Geometry
if is_ssh_mode is False:
    screen_geometry_dict = gfuncs.get_screen_geometry()
    imshow_x = int(screen_geometry_dict["margin_length"]["left_pixels"] * 1.05)
    imshow_y = int(screen_geometry_dict["margin_length"]["top_pixels"] * 1.05)


# Import Options
if agent_type == "dynamic":
    import options_iitp_third_year_demo_dynamic_night as options
    # import options_iitp_third_year_demo_dynamic_day as options
elif agent_type == "static":
    import options_iitp_interim_static as options
else:
    assert 0, "Agent Type Undefined type"

# Call Option Class
opts = options.option_class(agent_type=agent_type, agent_name=agent_name)

# Call Detection Framework (Detection Model)
detector_framework = snu_det.load_model(opts)

# Call Action Classifier Framework (Aclassifier Model)
aclassifier_framework = snu_acl.load_model(opts)


# Runner Functions
## Documentation for a function.
#
#  More details.
def snu_detector(framework, imgStruct_dict, opts):
    # Start Time for Detector
    DET_START_TIME = datetime.datetime.now()

    # Activate Detector Module (RefineDet)
    dets, thermal_boxes = snu_det.detect(framework, imgStruct_dict, opts)
    confs, labels = dets[:, 4:5], dets[:, 5:6]
    dets = dets[:, 0:4]

    # Remove Too Small Detections
    keep_indices = []
    for det_idx, det in enumerate(dets):
        if det[2]*det[3] >= opts.detector.tiny_area_threshold:
            keep_indices.append(det_idx)
    dets = dets[keep_indices, :]
    confs = confs[keep_indices, :]
    labels = labels[keep_indices, :]

    # Stop Time
    DET_STOP_TIME = datetime.datetime.now()

    # Detection Dictionary
    detections = {'dets': dets, 'confs': confs, 'labels': labels}

    return detections, thermal_boxes, (DET_STOP_TIME - DET_START_TIME).total_seconds()


# Multimodal Multi-target Tracker Runner
## Documentation for a function.
#
#  More details.
def snu_tracker(imgStruct_dict, fidx, detections, trackers, tracker_cands, opts, max_id):
    # Start Time for Multimodal Multi-target Tracker (MMT)
    MMT_START_TIME = datetime.datetime.now()

    # Activate MMT Module
    trackers, tracker_cands, max_id = \
        snu_mmt.tracker(imgStruct_dict, fidx, detections, max_id, opts, trackers, tracker_cands)

    # Stop Time for MMT
    MMT_STOP_TIME = datetime.datetime.now()

    return trackers, tracker_cands, max_id, (MMT_STOP_TIME - MMT_START_TIME).total_seconds()


# Action Classifier Runner
## Documentation for a function.
#
#  More details.
def snu_aclassifier(framework, imgStruct_dict, trackers, opts):
    # Start Time for Action Classifier
    ACL_START_TIME = datetime.datetime.now()

    # Activate ACL Module (Edit this, will be added)
    # trackers = snu_acl.aclassify_night(framework, imgStruct_dict, trackers, opts)
    trackers = snu_acl.aclassify(framework, imgStruct_dict, trackers, opts)

    # Stop Time for ACL
    ACL_STOP_TIME = datetime.datetime.now()

    return trackers, (ACL_STOP_TIME - ACL_START_TIME).total_seconds()
##############################################


############## Visualization Runners ##############
# Detection Result Visualization Runner
## Documentation for a function.
#
#  More details.
def visualize_detections(draw_rgb_frame, detections, opts):
    for det in detections['dets']:
        det = det.astype(np.int32)

        # Draw Rectangle BBOX
        cv2.rectangle(draw_rgb_frame,
                      (det[0], det[1]), (det[2], det[3]),
                      opts.visualization.detection['bbox_color'],
                      opts.visualization.detection['linewidth'])


# Tracklet Bounding Box and Action Classification Result Visualization
## Documentation for a function.
#
#  More details.
def visualize_tracklets(draw_rgb_frame, trks, opts):
    for trk in trks:
        state_bbox, _ = snu_mmt.fbbox.zx_to_bbox(trk.states[-1])
        state_bbox = state_bbox.astype(np.int32)

        # Draw Rectangle BBOX
        cv2.rectangle(draw_rgb_frame,
                      (state_bbox[0], state_bbox[1]), (state_bbox[2], state_bbox[3]),
                      (trk.color[0], trk.color[1], trk.color[2]),
                      opts.visualization.tracking['linewidth'])
        # Tracking Visualization Configuration
        font = opts.visualization.font
        font_size = opts.visualization.font_size
        pad_pixels = opts.visualization.pad_pixels
        info_interval = opts.visualization.info_interval

        # Visualize Tracklet ID
        trk_id_str = "id:" + str(trk.id) + ""
        (tw, th) = cv2.getTextSize(trk_id_str, font, fontScale=font_size, thickness=2)[0]
        text_x = int((state_bbox[0] + state_bbox[2]) / 2.0 - tw / 2.0)
        text_y = int(state_bbox[1] + th)
        box_coords = ((int(text_x - pad_pixels / 2.0), int(text_y - th - pad_pixels / 2.0)),
                      (int(text_x + tw + pad_pixels / 2.0), int(text_y + pad_pixels / 2.0)))
        cv2.rectangle(draw_rgb_frame, box_coords[0], box_coords[1], (trk.color[0], trk.color[1], trk.color[2]), cv2.FILLED)
        # cv2.putText receives "bottom-left" point of the text
        cv2.putText(draw_rgb_frame, trk_id_str, (text_x, text_y), font, font_size,
                    (255 - trk.color[0], 255 - trk.color[1], 255 - trk.color[2]), thickness=2)

        # Visualize Tracklet Depth (tentative, will be changed)
        if trk.depth is not None and opts.visualization.aclassifier["is_draw"] is False:
            trk_depth_str = "d=" + str(round(trk.depth, 3)) + "(m)"
            (tw, th) = cv2.getTextSize(trk_depth_str, font, fontScale=1.2, thickness=2)[0]
            text_x = int((state_bbox[0] + state_bbox[2]) / 2.0 - tw / 2.0)
            text_y = int((state_bbox[1] + state_bbox[3]) / 2.0 - th / 2.0)

            # Put Depth Text (Tentative)
            cv2.putText(draw_rgb_frame, trk_depth_str, (text_x, text_y), font, 1.2,
                        (255 - trk.color[0], 255 - trk.color[1], 255 - trk.color[2]), thickness=2)

        # Visualize Action Recognition Result
        if trk.pose is not None and opts.visualization.aclassifier['is_draw'] is True:
            if trk.label == 1:
                H = draw_rgb_frame.shape[0]
                W = draw_rgb_frame.shape[1]

                # Put Action Result text in words
                if trk.pose == 1:
                    pose_word = "Lie"
                elif trk.pose == 2:
                    pose_word = "Sit"
                elif trk.pose == 3:
                    pose_word = "Stand"
                else:
                    pose_word = "NAN"

                # cv2.putText(draw_rgb_frame_acl, str(int(trk.pose)), (min(int(trk.x[0] + (trk.x[4] / 2)), W - 1), min(int(trk.x[1] + (trk.x[5] / 2)), H - 1)),
                #             font, 3, (255 - trk.color[0], 255 - trk.color[1], 255 - trk.color[2]), thickness=3)

                cv2.putText(draw_rgb_frame, pose_word, (min(int(trk.x[0] + (trk.x[4] / 2)), W - 1), min(int(trk.x[1] + (trk.x[5] / 2)), H - 1)),
                            font, 1.5, (255 - trk.color[0], 255 - trk.color[1], 255 - trk.color[2]), thickness=3)
            ###################################################
###################################################


# Conditional Imports and Variable Initialization
if is_ros_embedded is True:
    # Import ROS Modules
    import rospy
    from rospy.numpy_msg import numpy_msg
    from cv_bridge import CvBridge, CvBridgeError

    # Import ROS Messages
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2
    from nav_msgs.msg import Odometry
    from osr_msgs.msg import Tracks

    # Import ROS Utility Modules
    import ros_utils

    # Import Rescue Module
    import rescue.force_depth_align as depth_align

    # Initialize Global Variables for the Multimodal Sensor Inputs via ROS
    d435_rgb_image, d435_depth_image = None, None
    d435_rgb_seq, d435_rgb_stamp, d435_depth_seq, d435_depth_stamp = None, None, None, None

    lidar1_image, lidar2_image, = None, None
    lidar1_seq, lidar1_stamp, lidar2_seq, lidar2_stamp = None, None, None, None

    infrared_image, thermal_image, nightvision_image = None, None, None
    infrared_seq, infrared_stamp, thermal_seq, thermal_stamp, nightvision_seq, nightvision_stamp = \
        None, None, None, None, None, None

    # Initialize LIDAR Point Cloud and Odometry Variables
    lidar_pointcloud, odometry = None, None

    # Get Camera Parameter (RGB, Depth)
    d435_rgb_cam_P, d435_depth_cam_P = None, None

    # Dummy Variables (just for initialization)
    sub_d435_rgb_cam_params, sub_d435_depth_cam_params = None, None
    # CvBridge Initialization
    bridge, bridge2 = CvBridge(), CvBridge()

    ############################
    # Callback Functions (ROS) #
    ############################
    # [1] D435 RGB
    ## Documentation for a function.
    #
    #  More details.
    def d435_rgb_callback(msg):
        global d435_rgb_image, d435_rgb_seq, d435_rgb_stamp
        d435_rgb_seq, d435_rgb_stamp = msg.header.seq, msg.header.stamp
        bgr_image = bridge.imgmsg_to_cv2(msg, opts.sensors.rgb["imgmsg_to_cv2_encoding"])

        if msg.encoding.__contains__("bgr") is True:
            d435_rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        elif msg.encoding.__contains__("rgb") is True:
            d435_rgb_image = bgr_image
        else:
            assert 0, "Unknown Image Encoding Type for Color Image"

    # [2] D435 Depth
    ## Documentation for a function.
    #
    #  More details.
    def d435_depth_callback(msg):
        global d435_depth_image, d435_depth_seq, d435_depth_stamp
        d435_depth_seq, d435_depth_stamp = msg.header.seq, msg.header.stamp
        d435_depth_image = bridge.imgmsg_to_cv2(msg, opts.sensors.depth["imgmsg_to_cv2_encoding"])
        # # Force Align
        if is_depth_not_aligned is True:
            d435_depth_image = depth_align.force_align_depth(d435_depth_image)

    # [3] LIDAR Image1 (calibrated w.r.t. D435 RGB Camera)
    ## Documentation for a function.
    #
    #  More details.
    def lidar1_callback(msg):
        global lidar1_image, lidar1_seq, lidar1_stamp
        lidar1_seq, lidar1_stamp = msg.header.seq, msg.header.stamp
        lidar1_image = bridge.imgmsg_to_cv2(msg, opts.sensors.lidar1["imgmsg_to_cv2_encoding"])

    # [3.1] LIDAR Image2 (calibrated w.r.t. Thermal Camera)
    ## Documentation for a function.
    #
    #  More details.
    def lidar2_callback(msg):
        global lidar2_image, lidar2_seq, lidar2_stamp
        lidar2_seq, lidar2_stamp = msg.header.seq, msg.header.stamp
        lidar2_image = bridge.imgmsg_to_cv2(msg, opts.sensors.lidar2["imgmsg_to_cv2_encoding"])

    # [3.2] LIDAR Point Cloud (WILL BE IMPLEMENTED)
    ## Documentation for a function.
    #
    #  More details.
    def lidar_pc_callback(msg):
        global lidar_pointcloud
        lidar_pointcloud = -1

    # [4] Infrared
    ## Documentation for a function.
    #
    #  More details.
    def infrared_callback(msg):
        global infrared_image, infrared_seq, infrared_stamp
        infrared_seq, infrared_stamp = msg.header.seq, msg.header.stamp
        infrared_image = bridge.imgmsg_to_cv2(msg, opts.sensors.infrared["imgmsg_to_cv2_encoding"])

    # [5] Thermal
    ## Documentation for a function.
    #
    #  More details.
    def thermal_callback(msg):
        global thermal_image, thermal_seq, thermal_stamp
        thermal_seq, thermal_stamp = msg.header.seq, msg.header.stamp
        thermal_image = bridge.imgmsg_to_cv2(msg, opts.sensors.thermal["imgmsg_to_cv2_encoding"])

    # [6] NightVision
    ## Documentation for a function.
    #
    #  More details.
    def nightvision_callback(msg):
        global nightvision_image, nightvision_seq, nightvision_stamp
        nightvision_seq, nightvision_stamp = msg.header.seq, msg.header.stamp
        nightvision_image = bridge.imgmsg_to_cv2(msg, opts.sensors.nightvision["imgmsg_to_cv2_encoding"])

    # Odometry Callback
    ## Documentation for a function.
    #
    #  More details.
    def odometry_callback(msg):
        global odometry
        odometry = msg

    # D435 RGB Camera Parameter Callback
    ## Documentation for a function.
    #
    #  More details.
    def d435_rgb_cam_param_callback(msg):
        global d435_rgb_cam_P, sub_d435_rgb_cam_params
        d435_rgb_cam_P = msg.P.reshape((3, 4))
        # sub_d435_rgb_cam_params.unregister()

    # D435 Depth Camera Parameter Callback
    ## Documentation for a function.
    #
    #  More details.
    def d435_depth_cam_param_callback(msg):
        global d435_depth_cam_P, sub_d435_depth_cam_params
        d435_depth_cam_P = msg.P.reshape((3, 4))
        # sub_d435_depth_cam_params.unregister()

else:
    curr_file_base_dir = os.path.dirname(os.path.abspath(__file__))
    sequence_base_dir = os.path.join(os.path.dirname(curr_file_base_dir), "DATA", "US_2019_POHANG", "01")
    results_save_base_dir = os.path.join(os.path.dirname(curr_file_base_dir), "results", "US_2019_POHANG", "01")

    sequence_name = "MV12_0604_d_fin"

    # make results save path for current sequence
    sequence_result_base_dir = os.path.join(results_save_base_dir, sequence_name)
    if os.path.isdir(sequence_result_base_dir) is False:
        os.mkdir(sequence_result_base_dir)

    # modal sequence paths
    modal_folder_subpaths = {
        "rgb": os.path.join(sequence_base_dir, sequence_name, "rgbdepth/color/"),           # 9141
        "disparity": os.path.join(sequence_base_dir, sequence_name, "rgbdepth/depth/"),     # 9141
        "infrared": os.path.join(sequence_base_dir, sequence_name, "rgbdepth/ir/"),         # 9140
        "nightvision": os.path.join(sequence_base_dir, sequence_name, "nv1/"),              # 9137
        "thermal": os.path.join(sequence_base_dir, sequence_name, "thermal/"),              # 9141
    }

    # modal sequence paths
    # [0]: filename lists
    # [1]: filepath lists
    modal_imgpath_list_dict = {
        "rgb": {
            "filenames": [],
            "filepaths": [],
        },
        "disparity": {
            "filenames": [],
            "filepaths": [],
        },
        "infrared": {
            "filenames": [],
            "filepaths": [],
        },
        "nightvision": {
            "filenames": [],
            "filepaths": [],
        },
        "thermal": {
            "filenames": [],
            "filepaths": [],
        },
    }

    for modal in modal_folder_subpaths.keys():
        results_sequence_modal_save_path = os.path.join(sequence_result_base_dir, modal)
        if os.path.isdir(results_sequence_modal_save_path) is False:
            os.mkdir(results_sequence_modal_save_path)

    for modal, img_seq_base_path in modal_folder_subpaths.items():
        curr_modal_img_name_list = os.listdir(img_seq_base_path)
        curr_modal_img_name_list.sort()
        modal_imgpath_list_dict[modal]["filenames"] = curr_modal_img_name_list

        curr_modal_img_path_list = [img_seq_base_path + filename for filename in curr_modal_img_name_list]
        modal_imgpath_list_dict[modal]["filepaths"] = curr_modal_img_path_list


# ROS Main Function
## Documentation for a function.
#
#  More details.
def ros_main():
    # Callback Global Variables
    global d435_rgb_image, d435_rgb_seq, d435_rgb_stamp
    global d435_depth_image, d435_depth_seq, d435_depth_stamp
    global lidar1_image, lidar1_seq, lidar1_stamp, lidar2_image, lidar2_seq, lidar2_stamp
    global infrared_image, infrared_seq, infrared_stamp
    global thermal_image, thermal_seq, thermal_stamp
    global nightvision_image, nightvision_seq, nightvision_stamp
    global lidar_pointcloud, odometry
    global d435_rgb_cam_P, d435_depth_cam_P
    global sub_d435_rgb_cam_params, sub_d435_depth_cam_params

    # Initialize Image Structs and Timestamp Structs
    imgStruct_dict, seqstamp_dict = {}, {}
    for modal, values in opts.sensors.__dict__.items():
        if "imgmsg_to_cv2_encoding" in values:
            imgStruct_dict[modal] = image_struct(modal, agent_type, agent_name=agent_name, is_ros_switch=is_ros_embedded)
            seqstamp_dict[modal] = ros_utils.seqstamp(modal)

            # For Static Agent Type, Update Camera Parameters Ahead
            if opts.agent_type == "static":
                cam_param = gfuncs.read_static_cam_param(
                    opts.paths["static_cam_param_path"], opts.agent_name)
                imgStruct_dict[modal].sensor_params.update_params(cam_param)
                imgStruct_dict[modal].sensor_params.get_camera_params()

    # ROS Node Initialization
    rospy.init_node("snu_module", anonymous=True)

    ########################### ROS Subscriber ###########################
    # [1] D435 RGB (Image)
    sub_rgb = rospy.Subscriber(opts.sensors.rgb["rostopic_name"], Image, d435_rgb_callback)
    # [2] D435 Depth (Image)
    sub_depth = rospy.Subscriber(opts.sensors.depth["rostopic_name"], Image, d435_depth_callback)
    # [3] LIDAR (Image and Pointcloud)
    sub_lidar1 = rospy.Subscriber(opts.sensors.lidar1["rostopic_name"], Image, lidar1_callback)
    sub_lidar2 = rospy.Subscriber(opts.sensors.lidar2["rostopic_name"], Image, lidar2_callback)
    sub_lidar_pc = rospy.Subscriber(opts.sensors.lidar_pc["rostopic_name"], PointCloud2, lidar_pc_callback)
    # [4] Infrared (Image)
    sub_infrared = rospy.Subscriber(opts.sensors.infrared["rostopic_name"], Image, infrared_callback)
    # [5] Thermal (Image)
    sub_thermal = rospy.Subscriber(opts.sensors.thermal["rostopic_name"], Image, thermal_callback)
    # [6] NightVision (Image)
    sub_nightvision = rospy.Subscriber(opts.sensors.nightvision["rostopic_name"], Image, nightvision_callback)

    # Subscribe Odometry
    sub_odometry = rospy.Subscriber(opts.sensors.odometry["rostopic_name"], Odometry, odometry_callback)

    # Subscribe Camera Info
    sub_d435_rgb_cam_params = rospy.Subscriber(opts.sensors.rgb["camerainfo_rostopic_name"], numpy_msg(CameraInfo), d435_rgb_cam_param_callback)
    sub_d435_depth_cam_params = rospy.Subscriber(opts.sensors.depth["camerainfo_rostopic_name"], numpy_msg(CameraInfo), d435_depth_cam_param_callback)
    ######################################################################

    ########################### ROS Publisher ###########################
    pub_tracks = rospy.Publisher(opts.publish_mesg["tracks"], Tracks, queue_size=1)
    pub_vis_img = rospy.Publisher(opts.publish_mesg["result_image"], Image, queue_size=1)
    #####################################################################

    # Initialize Tracklet and Tracklet Candidate
    trackers, tracker_cands = [], []

    # Initialize Frame Index, Tracklet Maximum ID
    fidx, max_id = 0, 0

    # Initialize Previous time-related Variables (modal-insensitive)
    prev_seqs = [None] * len(imgStruct_dict)

    # Initialize LIDAR image time-related variables
    lidar1_prev_seq, lidar2_prev_seq = None, None

    # Initial Sensor Timestamp Compensation
    initial_target_stamps_dict = {
        "rgb": None,
        "depth": None,
        "infrared": None,
        "thermal": None,
        "nightvision": None
    }

    ############ Practical Algorithm Starts from HERE ############
    while not rospy.is_shutdown():
        ############################# < Pre-processing > #############################
        ################# Image and Timestamp Synchronization #################

        # Timestamp List
        sync_target_stamps_dict = {
            "rgb": d435_rgb_stamp,
            "depth": d435_depth_stamp,
            "infrared": infrared_stamp,
            "thermal": thermal_stamp,
            "nightvision": nightvision_stamp
        }

        # Fill in Initial Sensor Time Dictionary
        if None in initial_target_stamps_dict.values():
            for key_idx, key in enumerate(initial_target_stamps_dict.keys()):
                if initial_target_stamps_dict[key] is None:
                    initial_target_stamps_dict[key] = sync_target_stamps_dict.values()[key_idx]

            if None in initial_target_stamps_dict.values():
                continue

        # Check if Sensor Stamps are Synchronized, do not run the algorithm until synchronized
        # Check which sensor stamps are synchronized, just indicate which sensors are asynchronous
        if None in sync_target_stamps_dict.values():
            continue
        else:
            sync_modal_list, is_all_synchronized, t_diff_matrix = \
                ros_utils.check_stamps_sync(sync_target_stamps_dict, initial_target_stamps_dict, opts.sensor_frame_rate)
            if is_all_synchronized is False:
                pass

        # LIDAR Sync
        if lidar1_image is not None:
            # If LIDAR Timestamp Increases
            if lidar1_seq > lidar1_prev_seq:
                lidar1_image = lidar1_image[:, :, 1]
            else:
                lidar1_image, lidar1_seq, lidar1_stamp = None, None, None

        if lidar2_image is not None:
            # If LIDAR Timestamp Increases
            if lidar2_seq > lidar2_prev_seq:
                lidar2_image = lidar2_image[:, :, 1]
            else:
                lidar2_image, lidar2_seq, lidar2_stamp = None, None, None

        lidar1_prev_seq, lidar2_prev_seq = lidar1_seq, lidar2_seq
        #######################################################################
        # Increase Frame Count Index
        fidx += 1

        print "Frame Index : %s" % str(fidx)

        ##################### Update Image Structs #####################
        for modal in imgStruct_dict:
            if modal == "rgb":
                sensor_image = copy.deepcopy(d435_rgb_image)
                seq = copy.copy(d435_rgb_seq)
                stamp = copy.deepcopy(d435_rgb_stamp)

                if opts.agent_type == "dynamic":
                    imgStruct_dict[modal].sensor_params = {
                        "P": d435_rgb_cam_P,
                    }
            elif modal == "depth":
                sensor_image = copy.deepcopy(d435_depth_image)
                seq = copy.copy(d435_depth_seq)
                stamp = copy.deepcopy(d435_depth_stamp)

                if opts.agent_type == "dynamic":
                    imgStruct_dict[modal].sensor_params = {
                        "P": d435_depth_cam_P,
                    }
            elif modal == "lidar1":
                sensor_image = copy.deepcopy(lidar1_image)
                seq = copy.copy(lidar1_seq)
                stamp = copy.deepcopy(lidar1_stamp)
            elif modal == "lidar2":
                sensor_image = copy.deepcopy(lidar2_image)
                seq = copy.copy(lidar2_seq)
                stamp = copy.deepcopy(lidar2_stamp)
            elif modal == "infrared":
                sensor_image = copy.deepcopy(infrared_image)
                seq = copy.copy(infrared_seq)
                stamp = copy.deepcopy(infrared_stamp)
            elif modal == "thermal":
                sensor_image = copy.deepcopy(thermal_image)
                seq = copy.copy(thermal_seq)
                stamp = copy.deepcopy(thermal_stamp)
            elif modal == "nightvision":
                sensor_image = copy.deepcopy(nightvision_image)
                seq = copy.copy(nightvision_seq)
                stamp = copy.deepcopy(nightvision_stamp)
            else:
                assert 0, "Unexpected modal [" + modal + "] Detected!"

            # Update
            seqstamp_dict[modal].update(seq, stamp)
            imgStruct_dict[modal].update_raw_frame(sensor_image, fidx, seqstamp_dict[modal])
        ###############################################################

        # Copy D435 RGB Camera Timestamp
        d435_rgb_timestamp = copy.deepcopy(imgStruct_dict['rgb'].seqstamp.timestamp)

        # Copy D435 RGB Image for Visualization
        draw_rgb_frame = copy.copy(imgStruct_dict['rgb'].frame.raw)

        # Convert Depth Image Type and Process (clip)
        if imgStruct_dict['depth'].frame.raw is not None:
            depth_img = imgStruct_dict['depth'].frame.raw.astype(np.float32)
            imgStruct_dict['depth'].frame.processed = \
                np.where((depth_img < opts.sensors.depth["clip_distance"]["min"]) |
                         (depth_img > opts.sensors.depth["clip_distance"]["max"]),
                         opts.sensors.depth["clip_value"], depth_img)
            del depth_img

        # Process (clip) LIDAR Images and convert to millimeter unit
        if 'lidar1' in imgStruct_dict:
            if imgStruct_dict['lidar1'].frame.raw is not None:
                lidar_img1 = imgStruct_dict['lidar1'].frame.raw * opts.sensors.lidar1['scaling_factor'] * 1000
                imgStruct_dict['lidar1'].frame.processed = \
                    np.where(lidar_img1 == 0, opts.sensors.lidar1['clip_value'], lidar_img1)
                del lidar_img1
        if 'lidar2' in imgStruct_dict:
            if imgStruct_dict['lidar2'].frame.raw is not None:
                lidar_img2 = imgStruct_dict['lidar2'].frame.raw * opts.sensors.lidar2['scaling_factor'] * 1000
                imgStruct_dict['lidar2'].frame.processed = \
                    np.where(lidar_img2 == 0, opts.sensors.lidar2['clip_value'], lidar_img2)
                del lidar_img2
        ########################### < Pre-processing Ends > ###########################

        ############################### < SNU Modules > ###############################
        ############# Detection Module #############
        detections, DET_TIME = snu_detector(detector_framework, imgStruct_dict, opts)
        ############################################

        ############# Multimodal Multi-target Tracking Module #############
        trackers, tracker_cands, max_id, MMT_TIME = \
            snu_tracker(imgStruct_dict, fidx, detections, trackers, tracker_cands, opts, max_id)
        ###################################################################

        ############# Action Classification Module #############
        trackers, ACL_TIME = snu_aclassifier(aclassifier_framework, imgStruct_dict, trackers, opts)
        ########################################################
        ###############################################################################

        ####################### < Visualization > #######################
        # [1] Detection Result Visualizer
        if opts.visualization.detection['is_draw'] is True:
            visualize_detections(draw_rgb_frame, detections, opts)
        # [2] Tracking and Action Classification Result Visualizer
        if opts.visualization.tracking['is_draw'] is True:
            visualize_tracklets(draw_rgb_frame, trackers, opts)
        #################################################################

        ############# Wrap Tracklets into ROS Topic Type #############
        out_tracks = ros_publish_wrapper(trackers, odometry, d435_rgb_timestamp)
        ##############################################################

        ################## Publish ROS Topics ##################
        # [1] Tracks
        pub_tracks.publish(out_tracks)
        # [2] SNU Result Image
        if opts.visualization.detection['is_draw'] is True or opts.visualization.tracking['is_draw'] is True:
            pub_vis_img.publish(bridge2.cv2_to_imgmsg(draw_rgb_frame, "rgb8"))
        ########################################################

        ################## Visualize ##################
        if is_ssh_mode is False:
            if opts.visualization.detection['is_draw'] is True or opts.visualization.tracking['is_draw'] is True:
                winname = "SNU Result"
                cv2.namedWindow(winname)
                if fidx == 1:
                    cv2.moveWindow(winname, imshow_x, imshow_y+50)
                # RGB to BGR (for bag files from agent)
                cv2.imshow(winname, cv2.cvtColor(draw_rgb_frame, cv2.COLOR_RGB2BGR))
                # cv2.imshow("ADEPTH", imgStruct_dict['depth'].frame.processed)
            cv2.waitKey(1)
        ###############################################


# Custom Image Sequence Run Main Function
## Documentation for a function.
#
#  More details.
def custom_main():
    # Target Modals
    target_modals = ["rgb", "disparity", "thermal"]

    imgStruct_dict = {}
    for modal in modal_imgpath_list_dict.keys():
        if modal in target_modals:
            if modal == "disparity":
                modal = "depth"
            imgStruct_dict[modal] = image_struct(modal, agent_type, agent_name=agent_name, is_ros_switch=is_ros_embedded)

    # Initialize Tracklet and Tracklet Candidate
    trackers, tracker_cands = [], []

    # Initialize Tracklet Maximum ID
    max_id = 0

    # Recursive Load Image Sequence (currently, load [1]RGB, [2]Disparity, [3]Thermal)
    modal_imgpath_list = [[]] * len(target_modals)
    modal_file_length_list = []
    for modal, img_file_list_dict in modal_imgpath_list_dict.items():
        if modal in target_modals:
            idx = target_modals.index(modal)
            modal_imgpath_list[idx] = img_file_list_dict["filepaths"]
            modal_file_length_list.append(len(img_file_list_dict["filepaths"]))

    # Check if all modal file lengths are same
    assert (modal_file_length_list.count(modal_file_length_list[0]) == len(modal_file_length_list)), \
        "Image File Lengths between modalities mismatched!"
    frame_length = modal_file_length_list[0]

    # Create Image Sequence Dictionaries
    target_modal_imgpath_dict = dict(zip(target_modals, modal_imgpath_list))

    # Target Modal Frame File Names
    target_modal_frame_file_name_dict = {}

    # For Image Sequence,
    start_frame_idx = 2319      # MV12_*_190604_220242_00.png
    end_frame_idx = 2839        # MV12_*_190604_220333_09.png
    for fidx in range(start_frame_idx, end_frame_idx):
        # Read Each Modal Images
        for modal, imgpath_list in target_modal_imgpath_dict.items():
            if modal == "rgb":
                modal_frame = cv2.cvtColor(cv2.imread(imgpath_list[fidx]), cv2.COLOR_BGR2RGB)

                if opts.agent_type == "dynamic":
                    imgStruct_dict[modal].sensor_params = {
                        "P": None
                    }

                target_modal_frame_file_name_dict[modal] = \
                    imgpath_list[fidx].split("/")[-1]

            elif modal == "disparity":
                modal = "depth"

                if opts.agent_type == "dynamic":
                    imgStruct_dict[modal].sensor_params = {
                        "P": None
                    }

                modal_frame = cv2.imread(imgpath_list[fidx], cv2.IMREAD_ANYDEPTH)

                target_modal_frame_file_name_dict[modal] = \
                    imgpath_list[fidx].split("/")[-1]

            elif modal == "thermal":
                modal_frame = cv2.imread(imgpath_list[fidx], cv2.IMREAD_GRAYSCALE)

                target_modal_frame_file_name_dict[modal] = \
                    imgpath_list[fidx].split("/")[-1]
            else:
                assert 0, "Unknown Modality [%s]" % modal

            # Update

            imgStruct_dict[modal].update_raw_frame(modal_frame, fidx, seqstamp=None)

        # Copy RGB Frame for Visualization
        draw_rgb_frame = copy.deepcopy(imgStruct_dict['rgb'].frame.raw)

        # Copy Thermal Frame for Visualization
        draw_thermal_frame = \
            cv2.cvtColor(copy.deepcopy(imgStruct_dict['thermal'].frame.raw), cv2.COLOR_GRAY2RGB)

        # Process Disparity Frame (1): Clip Distance
        if imgStruct_dict['depth'].frame.raw is not None:
            depth_img = imgStruct_dict['depth'].frame.raw.astype(np.float32)
            imgStruct_dict['depth'].frame.processed = \
                np.where((depth_img < opts.sensors.depth["clip_distance"]["min"]) |
                         (depth_img > opts.sensors.depth["clip_distance"]["max"]),
                         opts.sensors.depth["clip_value"], depth_img)
            del depth_img

        ############################### < SNU Modules > ###############################
        ############# Detection Module #############
        detections, thermal_boxes, DET_TIME = snu_detector(detector_framework, imgStruct_dict, opts)
        # detection coords ==>> Thermal Coordinates
        ############################################

        ############# Multimodal Multi-target Tracking Module #############
        trackers, tracker_cands, max_id, MMT_TIME = \
            snu_tracker(imgStruct_dict, fidx, detections, trackers, tracker_cands, opts, max_id)
        # tracklet (trackers) state: RGB Coordinates
        ###################################################################

        ############# Action Classification Module #############
        trackers, ACL_TIME = snu_aclassifier(aclassifier_framework, imgStruct_dict, trackers, opts)
        ########################################################
        ###############################################################################

        ####################### < Visualization > #######################
        # [1] Detection Result Visualizer
        if opts.visualization.detection['is_draw'] is True:
            thermal_detections = copy.deepcopy(detections)
            thermal_detections["dets"] = thermal_boxes

            visualize_detections(draw_thermal_frame, thermal_detections, opts)
            visualize_detections(draw_rgb_frame, detections, opts)

        # [2] Tracking and Action Classification Result Visualizer
        if opts.visualization.tracking['is_draw'] is True or opts.visualization.aclassifier["is_draw"] is True:
            visualize_tracklets(draw_rgb_frame, trackers, opts)

            thermal_trackers = copy.deepcopy(trackers)
            for th_trk_idx, th_trk in enumerate(thermal_trackers):
                th_trk.x = fbbox.bbox_to_zx(th_trk.x_bbox_thermal_aclassify, (0, 0))
                th_trk.states[-1] = th_trk.x
                thermal_trackers[th_trk_idx] = th_trk
                del th_trk

            visualize_tracklets(draw_thermal_frame, thermal_trackers, opts)
        #################################################################

        # Save Image Sequences
        if is_save_img_seq is True:
            # RGB and Thermal Frame File Names
            rgb_frame_file_name = ("[Frame_%05d]__" % fidx) + target_modal_frame_file_name_dict["rgb"]
            thermal_frame_file_name = ("[Frame_%05d]__" % fidx) + target_modal_frame_file_name_dict["thermal"]

            # RGB
            rgb_frame_file_path = os.path.join(sequence_result_base_dir, "rgb", rgb_frame_file_name)
            if (os.path.isfile(rgb_frame_file_path) is False) or (is_save_override is True):
                cv2.imwrite(rgb_frame_file_path, cv2.cvtColor(draw_rgb_frame, cv2.COLOR_RGB2BGR))

            # Thermal
            thermal_frame_file_path = os.path.join(sequence_result_base_dir, "thermal", thermal_frame_file_name)
            if os.path.isfile(thermal_frame_file_path) is False or (is_save_override is True):
                cv2.imwrite(thermal_frame_file_path, cv2.cvtColor(draw_thermal_frame, cv2.COLOR_RGB2BGR))

        ################## Visualize ##################
        if is_ssh_mode is False:
            if opts.visualization.detection['is_draw'] is True or opts.visualization.tracking['is_draw'] is True:
                imshow_window_names = {
                    "rgb": "SNU RGB Result",
                    "thermal": "SNU Thermal Result"
                }
                cv2.namedWindow(imshow_window_names["rgb"]), cv2.namedWindow(imshow_window_names["thermal"])

                if fidx == 1:
                    cv2.moveWindow(imshow_window_names["rgb"], imshow_x, imshow_y + 50)
                    cv2.moveWindow(imshow_window_names["thermal"], imshow_x+draw_rgb_frame.shape[1], imshow_y + 50)

                # Imshow RGB Images after converting to BGR format
                cv2.imshow(imshow_window_names["rgb"], cv2.cvtColor(draw_rgb_frame, cv2.COLOR_RGB2BGR))
                cv2.imshow(imshow_window_names["thermal"], cv2.cvtColor(draw_thermal_frame, cv2.COLOR_RGB2BGR))

            cv2.waitKey(1)
        ###############################################
        pass


# Main Namespace (if this script is executed, code starts from here)
if __name__ == '__main__':
    if is_ros_embedded is True:
        ros_main()
    else:
        custom_main()
























































































