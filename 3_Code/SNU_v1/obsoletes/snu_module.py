"""
SNU Detection and MOT Module v1.0 (ProtoType)

    Detection Module
        - Code written/modified by: ??? (xyz@qwerty.poi)

    MOT Module
        - Code written/modified by: Kyuewang Lee (kyuewang5056@gmail.com)

    Code Environment
        - python2.7
        - tensorflow==1.5.0
        - CUDA 9.0
            - cuDNN 7.0.5
        - ROS-kinetics

        < Dependencies >
            - [scikit-learn], [scikit-image], [FilterPy]
            - [numpy], [numba], [scipy], [matplotlib], [opencv-python]
"""

# Import Modules
import numpy as np
import argparse
import os
import sys
import datetime
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import tensorflow as tf
import cv2

sys.path.append('lib')

# Import SNU Modules
import src.snu_det_mot_module.scripts.mot_module as mot
from src.snu_det_mot_module.scripts.mot_class import TrackletCandidate, Tracklet
import src.snu_det_mot_module.scripts.obsoletes.detection_model as dmodel
from lib.config import config
from lib import dataset
from lib.utils.py_faster_rcnn_utils.cython_nms import nms
from src.snu_det_mot_module.scripts.lighthead_rcnn.lib.detection_opr.box_utils.box import DetBox
from lib.detection_opr.utils.bbox_transform import clip_boxes, bbox_transform_inv

# Set Detection Model Options
model_base_path = os.path.dirname(os.path.abspath(__file__))
model_path = model_base_path + "/model/detector.ckpt"
tfconfig = tf.ConfigProto(allow_soft_placement=True)
tfconfig.gpu_options.allow_growth = True
gpu_id = '0'

# Load Detection Model
infer_func, inputs = dmodel.load_model(model_path, tfconfig, gpu_id)

# MOT Parameters
trk_cont_destroy = 5
trk_cand_cont_destroy = 4

# Association Thresholds
cost_thresh = 0.5

# New Tracklet Association Age
gen_trk_asso_age = 3

# Detector Mode
is_snu_detector = True

# Visualization Options
is_vis_det = False
is_vis_mot = True

# Save Visualization Options
VIS_SAVE_BASE_PATH = "../../../../snu_module_result/"
is_save_fig = False


# Parse Arguments
def parse_args():
    parser = argparse.ArgumentParser(description='Unmanned_SNU_Module')
    # ADD-OPTIONS-HERE
    # parser.add_argument('--display', dest='display', \
    # help='Display online tracker output (slow) [False]',action='store_true'))

    args = parser.parse_args()

    return args


# Detector
def snu_detector(image):
    # image:
    #   - input data, image

    ori_shape = image.shape

    if config.eval_resize == False:
        resized_img, scale = image, 1
    else:
        resized_img, scale = dataset.resize_img_by_short_and_max_size(
            image, config.eval_image_short_size, config.eval_image_max_size)
    height, width = resized_img.shape[0:2]

    resized_img = resized_img.astype(np.float32) - config.image_mean
    resized_img = np.ascontiguousarray(resized_img[:, :, [2, 1, 0]])

    im_info = np.array(
        [[height, width, scale, ori_shape[0], ori_shape[1], 0]],
        dtype=np.float32)

    feed_dict = {inputs[0]: resized_img[None, :, :, :], inputs[1]: im_info}

    _, scores, pred_boxes, rois = infer_func(feed_dict=feed_dict)

    boxes = rois[:, 1:5] / scale

    # if cfg.TEST.BBOX_REG:
    pred_boxes = bbox_transform_inv(boxes, pred_boxes)
    pred_boxes = clip_boxes(pred_boxes, ori_shape)

    pred_boxes = pred_boxes.reshape(-1, config.num_classes, 4)
    result_boxes = []
    for j in range(1, config.num_classes):
        inds = np.where(scores[:, j] > config.test_cls_threshold)[0]
        cls_scores = scores[inds, j]
        cls_bboxes = pred_boxes[inds, j, :]
        cls_dets = np.hstack((cls_bboxes, cls_scores[:, np.newaxis])).astype(
            np.float32, copy=False)

        keep = nms(cls_dets, config.test_nms)
        cls_dets = np.array(cls_dets[keep, :], dtype=np.float, copy=False)
        for i in range(cls_dets.shape[0]):
            db = cls_dets[i, :]
            dbox = DetBox(
                db[0], db[1], db[2] - db[0], db[3] - db[1],
                tag=config.class_names[j], score=db[-1])

            result_boxes.append(dbox)

    if len(result_boxes) > config.test_max_boxes_per_image:
        result_boxes = sorted(
            result_boxes, reverse=True, key=lambda t_res: t_res.score) \
            [:config.test_max_boxes_per_image]

    temp_result_boxes = []
    for box in result_boxes:
        if box.score > config.test_vis_threshold and box.tag == 'person':
            x, y, w, h = box.x, box.y, box.w, box.h
            tag = box.tag
            score = box.score
            dbox = dict()
            dbox['x'] = x
            dbox['y'] = y
            dbox['w'] = w
            dbox['h'] = h
            dbox['d'] = -1
            dbox['class'] = tag
            dbox['score'] = score
            temp_result_boxes.append(dbox)

    result_dict = dict()
    result_dict['result_boxes'] = temp_result_boxes
    return result_dict


# Multiple Object Tracker
def snu_tracker(img, frame_idx, dets, trks=[], trk_cands=[]):
    # Initialize New Tracklet Variable
    new_trks = []

    # Destroy Prolonged Consecutively Unassociated Tracklets
    destroy_trk_indices = []
    for trk_idx, trk in enumerate(trks):
        consec_unasso = mot.get_max_consecutive(trk.is_associated, False)
        if consec_unasso == trk_cont_destroy:
            destroy_trk_indices.append(trk_idx)
    trks = mot.exclude_from_list(trks, destroy_trk_indices)

    # Destroy Prolonged Tracklet Candidates
    destroy_trkc_indices = []
    for trkc_idx, trk_cand in enumerate(trk_cands):
        if trk_cand.age == trk_cand_cont_destroy:
            destroy_trkc_indices.append(trkc_idx)
    trk_cands = mot.exclude_from_list(trk_cands, destroy_trkc_indices)

    # Associate Detections with Tracklets
    if len(trks) != 0:
        # Get Association Match INFO
        matches_d2t, unasso_det_indices, unasso_trk_indices = \
            mot.asso_det_tracklet(dets, trks, cost_thresh)
        # matches_d2t, unasso_det_indices, unasso_trk_indices = \
        #     mot.asso_det_tracklet_test(dets, trks, cost_thresh)

        # Update Associated Tracklets
        for match in matches_d2t:
            matched_detection = dets[match[0]]
            matched_trk = trks[match[1]]

            # Update
            matched_trk.update(matched_detection)
            trks[match[1]] = matched_trk
            del matched_trk

        # Update Unassociated Tracklets
        for unasso_trk_idx in unasso_trk_indices:
            unasso_trk = trks[unasso_trk_idx]

            # Update
            unasso_trk.update()
            trks[unasso_trk_idx] = unasso_trk
            del unasso_trk

        # Remove Associated Detections
        residual_dets = np.empty((len(unasso_det_indices), 4))
        for residual_det_idx, unasso_det_idx in enumerate(unasso_det_indices):
            residual_dets[residual_det_idx, :] = dets[unasso_det_idx]
        dets = residual_dets

    # Associate Residual Detections with Tracklet Candidates
    if len(trk_cands) == 0:
        for det in dets:
            new_trk_cand = TrackletCandidate(det)
            trk_cands.append(new_trk_cand)
            del new_trk_cand
    else:
        # Get Association Matches
        matches_d2tc, unasso_det_indices, unasso_trkc_indices = \
            mot.asso_det_trkcand(dets, trk_cands, cost_thresh)

        # Associate and Update Tracklet Candidates
        for match in matches_d2tc:
            matched_detection = dets[match[0]]
            matched_trk_cand = trk_cands[match[1]]

            # Update
            matched_trk_cand.update(matched_detection)
            trk_cands[match[1]] = matched_trk_cand
            del matched_trk_cand

        # # Destroy Unassociated Tracklet Candidates
        # trk_cands = mot.exclude_from_list(trk_cands, unasso_trkc_indices)

        # Update Unassociated Tracklet Candidates
        for unasso_trkc_idx in unasso_trkc_indices:
            unasso_trk_cand = trk_cands[unasso_trkc_idx]

            # Update
            unasso_trk_cand.update()
            trk_cands[unasso_trkc_idx] = unasso_trk_cand
            del unasso_trk_cand

        # Generate New Tracklet Candidates with the unassociated detections
        for unasso_det_idx in unasso_det_indices:
            new_trk_cand = TrackletCandidate(dets[unasso_det_idx])
            trk_cands.append(new_trk_cand)
            del new_trk_cand

    # Get Current Frame Maximum Tracklet ID
    max_id = mot.get_maximum_id(trks)

    # Generate New Tracklets from Tracklet Candidates
    if len(trk_cands) != 0:
        # Associate Only Tracklet Candidates with Detection Associated Consecutively for k frames
        selected_trkc_indices = []
        for trkc_idx, trk_cand in enumerate(trk_cands):
            max_consec_asso = mot.get_max_consecutive(trk_cand.is_associated, True)
            if max_consec_asso == gen_trk_asso_age:
                selected_trkc_indices.append(trkc_idx)
        sel_trk_cands = mot.select_from_list(trk_cands, selected_trkc_indices)

        # Initialize Tracklets
        for sel_trkc_idx, sel_trk_cand in enumerate(sel_trk_cands):
            selected_trkc_bbox, _ = mot.zx_to_bbox(sel_trk_cand.z[-1])

            # Generate Tracklet
            tracklet = Tracklet(selected_trkc_bbox, frame_idx, max_id + 1 + sel_trkc_idx)
            new_trks.append(tracklet)
            del tracklet
        del sel_trk_cands

        # Destroy Associated Tracklet Candidates
        trk_cands = mot.exclude_from_list(trk_cands, selected_trkc_indices)

    # Append New Tracklets
    for new_trk in new_trks:
        trks.append(new_trk)
        del new_trk
    del new_trks

    # # Sort Tracklets
    # trks = mot.sort_tracklets(trks)

    # Predict Tracklet States
    for trk_idx, trk in enumerate(trks):
        trk.predict()
        trks[trk_idx] = trk

    # MOT Message
    mot_mesg = "[Frame: " + str(fidx) + "] Tracking " + str(len(trks)) + " Tracklets..!"
    print(mot_mesg)

    return trks, trk_cands


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


# Main Function
if __name__ == '__main__':
    # Sequence Path
    seq_basepath = '../DATA/'
    sequences = ['ETH-Crossing', 'KITTI-17']

    # Make Output Directory
    if not os.path.exists('../output'):
        os.makedirs('../output')

    # For every image sequences
    for seq_num, seq in enumerate(sequences):
        # Sequence Number
        seq_num += 1

        # Current Sequence Path
        seqpath = seq_basepath + seq

        # Current Detection and Image Sequence Paths
        seq_detpath = seqpath + '/det'
        seq_imgspath = seqpath + '/imgs/'

        # Load Image Sequence Lists
        imgNameList = os.listdir(seq_imgspath)
        imgList = [seq_imgspath + filename for filename in imgNameList]
        imgList.sort()

        # Load Detections Prior
        # --> this part will be exterminated when the detection module is combined!
        seq_dets = np.loadtxt(seq_detpath + '/det.txt', delimiter=',')

        # Convert Detection Format (Convert to x1,y1,x2,y2)
        seq_dets[:, 4:6] += seq_dets[:, 2:4]

        # Visualization Figure
        if (is_vis_det is True) or (is_vis_mot is True):
            plt.ion()
            fig = plt.figure()

        # Generate Save Path
        if is_save_fig is True:
            VIS_SAVE_PATH = VIS_SAVE_BASE_PATH + seq + "/"
            if not os.path.exists(VIS_SAVE_PATH):
                os.makedirs(VIS_SAVE_PATH)

        # For every frames in the image sequence
        with open('../output/%s.txt' % seq, 'w') as out_file:
            print("Processing Sequence [%s]" % seq)

            # Initialize Tracklets and Tracklet Candidates
            trackers = []
            tracker_cands = []

            # Tracklet Color (Visualization)
            trk_colors = np.random.rand(32, 3)

            # For Current Frame Image
            for fidx, frameName in enumerate(imgList):
                # Frame Index Starts from 1
                fidx += 1

                # Load Current Frame Image
                # frame = io.imread(frameName)
                frame = cv2.imread(frameName)

                # Visualization Window
                if (is_vis_det is True) or (is_vis_mot is True):
                    ax1 = fig.add_subplot(111, aspect='equal')
                    ax1.imshow(frame)
                    plt.title('Sequence [' + seq + ']')

                # Start Timestamp for DETECTION
                DET_TS_START = datetime.datetime.now()
                # Load Detection (later, change this to the DETECTION MODULE)
                curr_dets2 = seq_dets[seq_dets[:, 0] == fidx, 2:6]

                # Detection Module
                result_dict = snu_detector(frame)
                result_boxes = result_dict['result_boxes']
                curr_dets = np.empty((len(result_boxes), 4))
                for idx, box in enumerate(result_boxes):
                    curr_dets[idx, 0] = box['x']
                    curr_dets[idx, 1] = box['y']
                    curr_dets[idx, 2] = box['x'] + box['w']
                    curr_dets[idx, 3] = box['y'] + box['h']

                # Select Detector
                if is_snu_detector is False:
                    curr_dets = curr_dets2

                # Stop Timestamp for DETECTION
                DET_TS_STOP = datetime.datetime.now()
                # Elapsed Time for the DETECTION MODULE (ms)
                DET_ELAPSED_TIME = (DET_TS_STOP-DET_TS_START).total_seconds() * 1000

                # Start Timestamp for MOT
                MOT_TS_START = datetime.datetime.now()
                # MOT Module
                trackers, tracker_cands = snu_tracker(frame, fidx, curr_dets, trackers, tracker_cands)
                # Stop Timestamp for MOT
                MOT_TS_STOP = datetime.datetime.now()
                # Elapsed Time for the MOT MODULE (ms)
                MOT_ELAPSED_TIME = (MOT_TS_STOP - MOT_TS_START).total_seconds() * 1000

                # Detection Visualization
                if is_vis_det is True:
                    ax1 = visualize_detections(curr_dets, ax1)

                # MOT Visualization
                if is_vis_mot is True:
                    ax1 = visualize_tracklets(trackers, trk_colors, ax1)

                # Draw Current Frame Result
                if (is_vis_det is True) or (is_vis_mot is True):
                    fig.canvas.flush_events()
                    plt.draw()
                    if ax1 is not None:
                        ax1.cla()

                # Save Figure Image
                if is_save_fig is True:
                    if (is_vis_det is True) or (is_vis_mot is True):
                        imgName = "result_%06d.png" % fidx
                        plt.savefig(imgName)
