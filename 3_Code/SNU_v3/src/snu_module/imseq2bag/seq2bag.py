#!/usr/bin/env python
"""
- Description PLZ

"""
import time
import os
import cv_bridge
import csv
from pandas import read_csv
import numpy as np
import rosbag
import rospy
import roslib
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import ImageFile
import cv2


class MODAL_BASE_OBJECT(object):
    def __init__(self, modal):
        # Modal
        self.modal = modal

        # Data
        self.data = None

        # Timestamp
        self.timestamp = None

    def __repr__(self):
        return self.modal

    def set_data(self, data, timestamp):
        self.data = data
        self.timestamp = timestamp

    def get_data(self):
        return {"data": self.data, "timestamp": self.timestamp}

    def get_modal(self):
        return self.modal


class IMAGE_MODAL_OBJ(MODAL_BASE_OBJECT):
    def __init__(self, modal):
        super(IMAGE_MODAL_OBJ, self).__init__(modal)


class LIDAR_MODAL_OBJ(MODAL_BASE_OBJECT):
    def __init__(self):
        super(LIDAR_MODAL_OBJ, self).__init__(modal="lidar")


class MODAL_DATA_OBJ(object):
    def __init__(self, modal_obj_list, modal):
        assert isinstance(modal_obj_list, list)

        # Modal
        self.modal = modal

        # Modal Object List
        self.modal_obj_list = modal_obj_list

    def __repr__(self):
        return self.modal

    def __len__(self):
        return len(self.modal_obj_list)

    def __getitem__(self, idx):
        return self.get_data(idx)

    def get_data(self, idx):
        return self.modal_obj_list[idx].get_data()


class MULTIMODAL_DATA_OBJ(object):
    def __init__(self, modal_data_obj_dict):
        assert isinstance(modal_data_obj_dict, dict)

        pre_modal_obj_len = None
        for modal, modal_data_obj in modal_data_obj_dict.items():
            if len(modal_data_obj) > 0:
                # FrameLength
                modal_obj_len = len(modal_data_obj)
                if pre_modal_obj_len is not None:
                    if pre_modal_obj_len != modal_obj_len:
                        raise AssertionError("Modal Images Length Mismatch!")
                pre_modal_obj_len = modal_obj_len

            # Set Class Attributes
            setattr(self, modal, modal_data_obj)

        # Set Data Length
        self.__dataLength = pre_modal_obj_len

    def __len__(self):
        return self.__dataLength

    def __getitem__(self, idx):
        raise NotImplementedError()

    def get_data(self, idx):
        data_dict, timestamp_dict = {}, {}
        for modal, modal_obj in self.__dict__.iteritems():
            if modal == "_MULTIMODAL_DATA_OBJ__dataLength":
                continue
            if len(modal_obj) < idx+1:
                data_dict[modal], timestamp_dict[modal] = None, None
                continue
            # Get Current Modal Data and Timestamp, Append to Dictionary
            idx_data_timestamp_dict = modal_obj.get_data(idx)
            data_dict[modal], timestamp_dict[modal] = \
                idx_data_timestamp_dict["data"], idx_data_timestamp_dict["timestamp"]
        return data_dict, timestamp_dict


def load_multimodal_data(base_path):
    # Check if Base Path Exists
    if os.path.isdir(base_path) is False:
        raise AssertionError()

    # Get Folder Lists
    modal_lists = os.listdir(base_path)
    if "xml" in modal_lists:
        modal_lists.remove("xml")

    # for each modalities
    modal_data_obj_dict = {}
    for modal in modal_lists:
        # Join Path
        curr_modal_base_path = os.path.join(base_path, modal)

        # Get Modal File Lists
        modal_obj_list = []
        curr_modal_file_lists = sorted(os.listdir(curr_modal_base_path))
        if len(curr_modal_file_lists) > 0:
            # Traverse through Files
            for filename in curr_modal_file_lists:
                # Current File Path
                curr_filepath = os.path.join(curr_modal_base_path, filename)

                # Initialize Modal Object
                if modal != "lidar":
                    modal_obj = IMAGE_MODAL_OBJ(modal=modal)
                else:
                    modal_obj = LIDAR_MODAL_OBJ()

                # Load Data
                if modal == "RGB":
                    data = cv2.cvtColor(cv2.imread(curr_filepath), cv2.COLOR_BGR2RGB)
                elif modal != "lidar":
                    data = cv2.imread(curr_filepath, -1)
                else:
                    data = read_csv(curr_filepath).values

                # Parse Timestamp
                _date = filename.split(".")[0].split("_")[2]
                _time = filename.split(".")[0].split("_")[3]
                _fidx = filename.split(".")[0].split("_")[4]
                timestamp = {"date": _date, "time": _time, "fidx": _fidx}

                # Append Data and Timestamp to Modal Object
                modal_obj.set_data(data=data, timestamp=timestamp)
                modal_obj_list.append(modal_obj)

        # Make Modal Data Object
        modal_data_obj = MODAL_DATA_OBJ(modal_obj_list=modal_obj_list, modal=modal)
        modal_data_obj_dict[modal] = modal_data_obj

    # Multimodal Data Object
    multimodal_data_obj = MULTIMODAL_DATA_OBJ(modal_data_obj_dict=modal_data_obj_dict)
    # multimodal_data_obj.get_data(0)

    return multimodal_data_obj


def generate_multimodal_bag_file(MMT_OBJ, base_path):
    # Get Bag File Name
    bag_name = os.path.split(base_path)[-1] + ".bag"

    # Initialize Bag File
    bag = rosbag.Bag(os.path.join(base_path, bag_name), "w")

    # Iterate for Multimodal Sensors
    try:
        for fidx in range(len(MMT_OBJ)):
            mmt_data_dict, mmt_timestamp_dict = MMT_OBJ.get_data(fidx)
            bridge = cv_bridge.CvBridge()
            for modal, modal_data in mmt_data_dict.items():
                # Define Each Modals' Encoding and Frame ID
                if modal == "RGB":
                    modal_encoding = "rgb8"
                    modal_frame_id = "/camera/" + modal.lower()
                elif modal == "aligned_depth":
                    modal_encoding = "mono16"
                    modal_frame_id = "/camera/" + modal.lower()
                elif modal == "thermal":
                    modal_encoding = "mono16"
                    modal_frame_id = "/camera/" + modal.lower()
                elif modal == "infrared":
                    modal_encoding = "mono8"
                    modal_frame_id = "/camera/" + modal.lower()
                elif modal == "nightvision":
                    modal_encoding = "rgb8"
                    modal_frame_id = "/camera/" + modal.lower()
                elif modal == "lidar":
                    # # # # #
                    continue
                else:
                    raise NotImplementedError()

                if modal_data is None:
                    pass
                else:
                    # Get Timestamp of the Current Modal
                    modal_timestamp = mmt_timestamp_dict[modal]
                    t_hour = int(modal_timestamp["time"][0:2])
                    t_min = int(modal_timestamp["time"][2:4])
                    t_sec = int(modal_timestamp["time"][4:])
                    t_seq = float(modal_timestamp["fidx"])

                    modal_stamp = rospy.rostime.Time.from_sec(
                        float(t_hour*3600+t_min*60+t_sec) + 0.1*t_seq
                    )

                    # Initialize ROS Message Type
                    if modal == "lidar":
                        pass
                    else:
                        ROS_MODAL_IMG = Image()
                        ROS_MODAL_IMG = bridge.cv2_to_imgmsg(modal_data, modal_encoding)
                        ROS_MODAL_IMG.header.seq = fidx
                        ROS_MODAL_IMG.header.stamp = modal_stamp
                        ROS_MODAL_IMG.header.frame_id = modal_frame_id

                        bag.write(modal_frame_id + "/image", ROS_MODAL_IMG, modal_stamp)
    finally:
        bag.close()


if __name__ == "__main__":
    base_path = "/home/snu/DATA/USR_4th_Final_Demo"
    MMT_DATA_OBJ = load_multimodal_data(base_path=base_path)
    generate_multimodal_bag_file(MMT_OBJ=MMT_DATA_OBJ, base_path=base_path)
    pass
