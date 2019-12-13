"""
Outdoor Unmanned Surveillance Robot SNU Multimodal MOT Module v1.5

    - Code partially based on SORT (ICIP2016)

    - Code written/modified by : Kyuewang Lee (kyuewang5056@gmail.com)

"""

from __future__ import print_function

import numpy as np
import mot_module as mot
from mot_class import TrackletCandidate, Tracklet


# Mutimodal MOT Tracker (ProtoType)
def tracker(color_img, depth_img, fidx, dets, motparams, trks=[], trk_cands=[]):
    # Unpack motparams
    unasso_trk_destroy = motparams.unasso_trk_destroy
    unasso_trkc_destroy = motparams.unasso_trkc_destroy
    cost_thresh = motparams.cost_thresh
    trkc_to_trk_asso_age = motparams.trkc_to_trk_asso_age
    dhist_bin = motparams.dhist_bin
    depth_clip_dist = motparams.DEPTH_CLIP_DIST

    # Initialize New Tracklet Variable
    new_trks = []

    # Destroy Prolonged Consecutively Unassociated Tracklets
    destroy_trk_indices = []
    for trk_idx, trk in enumerate(trks):
        consec_unasso = mot.get_max_consecutive(trk.is_associated, False)
        if consec_unasso == unasso_trk_destroy:
            destroy_trk_indices.append(trk_idx)
    trks = mot.exclude_from_list(trks, destroy_trk_indices)

    # Destroy Prolonged Tracklet Candidates
    destroy_trkc_indices = []
    for trkc_idx, trk_cand in enumerate(trk_cands):
        if trk_cand.age == unasso_trkc_destroy:
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
            if max_consec_asso == trkc_to_trk_asso_age:
                selected_trkc_indices.append(trkc_idx)
        sel_trk_cands = mot.select_from_list(trk_cands, selected_trkc_indices)

        # Initialize Tracklets
        for sel_trkc_idx, sel_trk_cand in enumerate(sel_trk_cands):
            selected_trkc_bbox, _ = mot.zx_to_bbox(sel_trk_cand.z[-1])

            # Generate Tracklet
            tracklet = Tracklet(selected_trkc_bbox, fidx, max_id + 1 + sel_trkc_idx)
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

    # Predict Tracklet States
    for trk_idx, trk in enumerate(trks):
        trk.predict()
        trks[trk_idx] = trk

    # Process 3-D via Depth Information
    depth_list = mot.depth_inference(trks, depth_img, dhist_bin)
    for depth_idx, depth in enumerate(depth_list):
        trk = trks[depth_idx]
        trk.update_depth(depth, depth_clip_dist)

    # MOT Message
    mot_mesg = "[Frame: " + str(fidx) + "] Tracking " + str(len(trks)) + " Tracklets..!"
    print(mot_mesg)

    return trks, trk_cands




















