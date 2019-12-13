## @package action_recognition_thermal.py
#  python file for action recognition (classification) using a action classifier model
#  specified on thermal image sequence
#
#  More details.
import cv2
import numpy as np
import torch
import torch.nn as nn
from torchvision import models, transforms


## Documentation for a function.
#
#  More details.
def load_model(device_n, model_dir):
    cuda_device_str = "cuda:" + str(device_n)
    device = torch.device(cuda_device_str if torch.cuda.is_available() else "cpu")

    model = torch.load(model_dir)
    model = model.to(device)

    return model


## Documentation for a function.
#
#  More details.
def voting(pose_list):
    counter = 0
    num = pose_list[0]
    for i in pose_list:
        curr_freq = pose_list.count(i)
        if curr_freq > counter:
            counter = curr_freq
            num = i
    return num+1


## Documentation for a function.
#
#  More details.
def aclassify(model, thermal_img, trackers, device_n) :
    cuda_device_str = "cuda:" + str(device_n)
    device = torch.device(cuda_device_str if torch.cuda.is_available() else "cpu")

    H = thermal_img.shape[0]
    W = thermal_img.shape[1]
    for tracker_idx, tracker in enumerate(trackers):
        a = max(0, int(tracker.x[1] - (tracker.x[5] / 2)))
        b = min(int(tracker.x[1] + (tracker.x[5] / 2)), H - 1)
        c = max(0, int(tracker.x[0] - (tracker.x[4] / 2)))
        d = min(int(tracker.x[0] + (tracker.x[4] / 2)), W - 1)

        if (a >= b) or (c >= d):
            try:
                tracker.poselist.insert(0, 0)
            except:
                pass
        else:
            crop_image = thermal_img[a:b, c:d]
            img_thermal_rs = cv2.resize(crop_image, dsize=(60, 60), interpolation=cv2.INTER_AREA)

            d = img_thermal_rs / 255.0
            d = d.reshape(60, 60, 1)
            x = transforms.ToTensor()(d)
            x = transforms.Normalize([0.677], [0.172])(x)
            x = (x.view(1, 1, 60, 60)).to(device)  #
            tmp = model(x)
            tracker.pose_list.insert(0, torch.max(tmp, 1)[1])

        trackers[tracker_idx] = tracker

        if len(tracker.pose_list) > 7:
            tracker.pose_list.pop()

        tracker.pose = voting(tracker.pose_list)
    return trackers
