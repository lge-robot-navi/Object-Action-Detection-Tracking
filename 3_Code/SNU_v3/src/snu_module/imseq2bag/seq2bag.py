#!/usr/bin/env python
"""
- Description PLZ

"""
import os
import numpy as np
import ros_numpy as ros_np
from pandas import read_csv
import rosbag
import rospy
import roslib
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2


_CAMERAINFO = {
    # RGB Camera Info
    "RGB": {
        # Distortion Matrix
        "D": np.array([0, 0, 0, 0, 0], dtype=np.float64).tolist(),
        # Intrinsic Camera Matrix
        "K": np.array([610.456482, 0, 325.258026, 0, 610.387329, 242.385468, 0, 0, 1],
                      dtype=np.float64).tolist(),
        # Rectification Matrix
        "R": np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=np.float64).tolist(),
        # Projection Matrix
        "P": np.array([610.456482, 0, 325.258026, 0, 0, 610.387329, 242.385468, 0, 0, 0, 1, 0],
                      dtype=np.float64).tolist()
    },

    # Depth
    "aligned_depth": {
        # Distortion Matrix
        "D": np.array([0, 0, 0, 0, 0], dtype=np.float64).tolist(),
        # Intrinsic Camera Matrix
        "K": np.array([386.904297, 0, 322.290070, 0, 386.904297, 243.228958, 0, 0, 1],
                      dtype=np.float64).tolist(),
        # Rectification Matrix
        "R": np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=np.float64).tolist(),
        # Projection Matrix
        "P": np.array([386.904297, 0, 322.290070, 0, 0, 386.904297, 243.228958, 0, 0, 0, 1, 0],
                      dtype=np.float64).tolist()
    },

    # Infrared
    "infrared": {
        # Distortion Matrix
        "D": np.array([0, 0, 0, 0, 0], dtype=np.float64).tolist(),
        # Intrinsic Camera Matrix
        "K": np.array([386.904297, 0, 322.290070, 0, 386.904297, 243.228958, 0, 0, 1],
                      dtype=np.float64).tolist(),
        # Rectification Matrix
        "R": np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=np.float64).tolist(),
        # Projection Matrix
        "P": np.array([386.904297, 0, 322.290070, 0, 0, 386.904297, 243.228958, 0, 0, 0, 1, 0],
                      dtype=np.float64).tolist()
    },

    # Nightvision
    "nightvision": {
        # Distortion Matrix
        "D": [],
        # Intrinsic Camera Matrix
        "K": [],
        # Rectification Matrix
        "R": [],
        # Projection Matrix
        "P": []
    },

    # Thermal
    "thermal": {
        # Distortion Matrix
        "D": np.array([0, 0, 0, 0, 0], dtype=np.float64).tolist(),
        # Intrinsic Camera Matrix
        "K": np.array([436.128240, 0, 342.138120, 0, 406.085860, 222.085950, 0, 0, 1],
                      dtype=np.float64).tolist(),
        # Rectification Matrix
        "R": np.array([1, 0, 0, 0, 1, 0, 0, 0, 1], dtype=np.float64).tolist(),
        # Projection Matrix
        "P": np.array([436.128240, 0, 342.138120, 0, 0, 406.085860, 222.085950, 0, 0, 0, 1, 0],
                      dtype=np.float64).tolist()
    }
}


class CAMERA_INFO_OBJECT(object):
    def __init__(self, modal, timestamp):
        assert isinstance(timestamp, dict)

        self.modal = modal
        self.header = Header()

        t_hour = int(timestamp["time"][0:2])
        t_min = int(timestamp["time"][2:4])
        t_sec = int(timestamp["time"][4:])
        t_seq = float(timestamp["fidx"])
        modal_stamp = rospy.rostime.Time.from_sec(
            float(t_hour * 3600 + t_min * 60 + t_sec) + 0.1 * t_seq
        )
        self.header.stamp = modal_stamp

        # Camera Parameters
        self.D, self.K, self.R, self.P = None, None, None, None

    def update_camera_parameters(self, D, K, R, P):
        self.D, self.K, self.R, self.P = D, K, R, P

    def get_camera_parameters(self):
        return {"D": self.D, "K": self.K, "R": self.R, "P": self.P}

    def get_timestamp(self):
        return self.header.stamp

    def to_CameraInfo(self, width, height, distortion_model="plumb_bob"):
        ros_camera_info = CameraInfo()
        ros_camera_info.header = self.header
        ros_camera_info.width = width
        ros_camera_info.height = height
        ros_camera_info.distortion_model = distortion_model

        ros_camera_info.D = self.D
        ros_camera_info.K = self.K
        ros_camera_info.R = self.R
        ros_camera_info.P = self.P

        ros_camera_info.binning_x, ros_camera_info.binning_y = 1, 1

        return ros_camera_info


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
    def __init__(self, modal, timestamp):
        super(IMAGE_MODAL_OBJ, self).__init__(modal)

        # CameraInfo
        self.camera_info = CAMERA_INFO_OBJECT(modal=self.modal, timestamp=timestamp)

    def get_camera_info(self):
        return self.camera_info

    def update_camera_parameters(self, D, K, R, P):
        self.camera_info.update_camera_parameters(D, K, R, P)

    def get_camera_parameters(self):
        return self.camera_info.get_camera_parameters()


class LIDAR_MODAL_OBJ(MODAL_BASE_OBJECT):
    def __init__(self):
        super(LIDAR_MODAL_OBJ, self).__init__(modal="lidar")

    @staticmethod
    def get_camera_info():
        return None

    @staticmethod
    def get_camera_parameters():
        return None


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

    def get_camera_info(self, idx):
        return self.modal_obj_list[idx].get_camera_info()

    def get_camera_parameters(self, idx):
        return self.modal_obj_list[idx].get_camera_parameters()


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

    def get_camera_info(self, idx):
        camera_info_dict = {}
        for modal, modal_obj in self.__dict__.iteritems():
            if modal == "_MULTIMODAL_DATA_OBJ__dataLength":
                continue
            if len(modal_obj) < idx+1:
                camera_info_dict[modal] = None
                continue
            idx_camera_info = modal_obj.get_camera_info(idx)
            camera_info_dict[modal] = idx_camera_info
        return camera_info_dict

    def get_camera_parameters(self, idx=0):
        camera_params_dict = {}
        for modal, modal_obj in self.__dict__.iteritems():
            if modal == "_MULTIMODAL_DATA_OBJ__dataLength":
                continue
            if len(modal_obj) < idx + 1:
                camera_params_dict[modal] = None
                continue
            idx_camera_params = modal_obj.get_camera_parameters(idx)
            camera_params_dict[modal] = idx_camera_params
        return camera_params_dict


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
