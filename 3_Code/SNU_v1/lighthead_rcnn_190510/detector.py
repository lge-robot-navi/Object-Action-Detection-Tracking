# encoding: utf-8
"""
@author: jemmy li
@contact: zengarden2009@gmail.com
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys, os
sys.path.append('lib')

from lib.config import config

from lib import dataset
from lib import network_desp
import tensorflow as tf
import numpy as np
import cv2, os, json

from lib.utils.py_faster_rcnn_utils.cython_nms import nms, nms_new
from lib.detection_opr.box_utils.box import DetBox
from lib.detection_opr.utils.bbox_transform import clip_boxes, bbox_transform_inv
from functools import partial

import time

def load_model(dev='0'):
    # dev:
    #   - device id, int
    #   - example: '0'

    model_path_root = os.path.dirname(os.path.abspath(__file__))
    model_file = model_path_root + '/model_dump/detector.ckpt'
    # print(model_file)

    os.environ["CUDA_VISIBLE_DEVICES"] = dev
    tfconfig = tf.ConfigProto(allow_soft_placement=True)
    tfconfig.gpu_options.allow_growth = True
    sess = tf.Session(config=tfconfig)

    net = network_desp.Network() # create network
    inputs = net.get_inputs() # create input node (placeholder)

    net.inference('TEST', inputs) # define forward path
    test_collect_dict = net.get_test_collection() # create result node
    test_collect = [it for it in test_collect_dict.values()]
    saver = tf.train.Saver()

    saver.restore(sess, model_file)
    return partial(sess.run, test_collect), inputs
    
infer_func, inputs = load_model()

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



######################################################################################
criterion = time.time()


cap = cv2.VideoCapture(0)
i = 0
while True:
    ret, image = cap.read()
    # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
# for i in range(1000):
    # image_path = 'sample.jpg'
    # image = cv2.imread(image_path)



    result_dict = snu_detector(image)  


    for box in (result_dict['result_boxes']):
        cv2.rectangle(image,(int(box['x']), int(box['y'])), (int(box['x']+box['w']), int(box['y']+box['h'])), (255,0,0), 4)
    cv2.imshow('result',image)
    cv2.waitKey(1)
    i += 1
    if i % 100 == 0 :
        print(i)
    if i == 999 :
        break
print('{} iter _ total_time : {} s'.format(i+1, time.time()-criterion ) )

######################################################################################