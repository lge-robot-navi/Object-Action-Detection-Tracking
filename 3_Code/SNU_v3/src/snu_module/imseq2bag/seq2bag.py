#!/usr/bin/env python
"""
- Description PLZ

"""
import os
import numpy as np
from pandas import read_csv
import rosbag
import rospy
import roslib
roslib.load_manifest('sensor_msgs')
from cv_bridge import CvBridge
import cv2

from mmt_params import CAMERA_INFO as _CAMERAINFO
from camera_objects import IMAGE_MODAL_OBJ, LIDAR_MODAL_OBJ, MODAL_DATA_OBJ, MULTIMODAL_DATA_OBJ


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

                # Parse Timestamp
                _date = filename.split(".")[0].split("_")[2]
                _time = filename.split(".")[0].split("_")[3]
                _fidx = filename.split(".")[0].split("_")[4]
                timestamp = {"date": _date, "time": _time, "fidx": _fidx}

                # Initialize Modal Object
                if modal != "lidar":
                    modal_obj = IMAGE_MODAL_OBJ(modal=modal, timestamp=timestamp)

                    # Update Camera Parameters
                    modal_camerainfo = _CAMERAINFO[modal]
                    modal_obj.update_camera_parameters(
                        D=modal_camerainfo["D"], K=modal_camerainfo["K"],
                        R=modal_camerainfo["R"], P=modal_camerainfo["P"]
                    )
                else:
                    modal_obj = LIDAR_MODAL_OBJ()

                # Load Data
                if modal == "RGB":
                    data = cv2.cvtColor(cv2.imread(curr_filepath), cv2.COLOR_BGR2RGB)
                elif modal != "lidar":
                    data = cv2.imread(curr_filepath, -1)
                else:
                    data = read_csv(curr_filepath).values

                # Append Data and Timestamp to Modal Object
                modal_obj.set_data(data=data, timestamp=timestamp)
                modal_obj_list.append(modal_obj)

        # Make Modal Data Object
        modal_data_obj = MODAL_DATA_OBJ(modal_obj_list=modal_obj_list, modal=modal)
        modal_data_obj_dict[modal] = modal_data_obj

    # Multimodal Data Object
    multimodal_data_obj = MULTIMODAL_DATA_OBJ(modal_data_obj_dict=modal_data_obj_dict)
    # _ = multimodal_data_obj.get_camera_parameters(0)

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
            mmt_camera_info_dict = MMT_OBJ.get_camera_info(fidx)
            bridge = CvBridge()
            for modal, modal_data in mmt_data_dict.items():
                # Define Each Modals' Encoding and Frame ID
                if modal == "RGB":
                    modal_encoding = "rgb8"
                    modal_frame_id = "osr/image_color"
                elif modal == "aligned_depth":
                    modal_encoding = "mono16"
                    modal_frame_id = "osr/image_" + modal.lower()
                elif modal == "thermal":
                    modal_encoding = "mono16"
                    modal_frame_id = "osr/image_" + modal.lower()
                elif modal == "infrared":
                    modal_encoding = "mono8"
                    modal_frame_id = "osr/image_ir"
                elif modal == "nightvision":
                    modal_encoding = "rgb8"
                    modal_frame_id = "osr/image_nv1"
                elif modal == "lidar":
                    modal_encoding = None
                    modal_frame_id = "osr/lidar_pointcloud"
                else:
                    raise NotImplementedError()

                # Get Each Modals' Camera Parameters
                modal_camera_info = mmt_camera_info_dict[modal]

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
                        # ROS_MODAL_IMG = Image()
                        ROS_MODAL_IMG = bridge.cv2_to_imgmsg(modal_data, modal_encoding)
                        ROS_MODAL_IMG.header.seq = fidx
                        ROS_MODAL_IMG.header.stamp = modal_stamp
                        ROS_MODAL_IMG.header.frame_id = modal_frame_id

                        # CameraInfo
                        MODAL_CAMERA_INFO = modal_camera_info.to_CameraInfo(
                            width=modal_data.shape[1], height=modal_data.shape[0]
                        )

                        bag.write(modal_frame_id + "/image", ROS_MODAL_IMG, modal_stamp)
                        if modal != "nightvision":
                            if modal == "aligned_depth":
                                bag.write("osr/image_depth_camerainfo", MODAL_CAMERA_INFO, modal_stamp)
                            else:
                                bag.write(modal_frame_id + "/image_camerainfo", MODAL_CAMERA_INFO, modal_stamp)

    finally:
        bag.close()
        pass


if __name__ == "__main__":
    base_path = "/home/snu/DATA/USR_4th_Final_Demo"
    MMT_DATA_OBJ = load_multimodal_data(base_path=base_path)
    generate_multimodal_bag_file(MMT_OBJ=MMT_DATA_OBJ, base_path=base_path)
    pass
