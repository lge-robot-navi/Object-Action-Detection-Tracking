"""
Unmanned Surveillance Robot SNU Module
Neural-Network Model (ProtoType)

    Code written by SNU Students

"""

from __future__ import division, print_function, absolute_import

import os
import tensorflow as tf
import json
from functools import partial

from lib import dataset
from lib import network_desp
from src.snu_det_mot_module.scripts.lighthead_rcnn.lib.detection_opr.box_utils.box import DetBox


def load_model(model_path, tfconfig, dev='0'):
    # dev:
    #   - device id, int
    #   - example: '0'

    os.environ["CUDA_VISIBLE_DEVICES"] = dev
    sess = tf.Session(config=tfconfig)

    net = network_desp.Network()  # create network
    inputs = net.get_inputs()  # create input node (placeholder)

    net.inference('TEST', inputs)  # define forward path
    test_collect_dict = net.get_test_collection()  # create result node
    test_collect = [it for it in test_collect_dict.values()]
    saver = tf.train.Saver()

    saver.restore(sess, model_path)
    return partial(sess.run, test_collect), inputs
