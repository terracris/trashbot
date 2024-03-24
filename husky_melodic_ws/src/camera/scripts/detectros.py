import argparse
import datetime
import time
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
# import realsense_depth
from numpy import random

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages, MyLoadImages
from utils.general import *
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized


class simulation_opt:
    def __init__(self,weights,img_size=640,conf_thres=0.80,iou_thres=0.45,device='',view_img=False,
                 classes=None,agnostic_nms=False,augment=False,update=False,exist_ok=False):
        self.weights=weights
        self.source=None
        self.img_size=img_size
        self.conf_thres=conf_thres
        self.iou_thres=iou_thres
        self.device=device
        self.view_img=view_img
        self.classes=classes
        self.agnostic_nms=agnostic_nms
        self.augment=augment
        self.update=update
        self.exist_ok=exist_ok


class detectapi:
    def __init__(self, weights, img_size=640):
        self.opt = simulation_opt(weights=weights, img_size=img_size)
        weights, imgsz = self.opt.weights, self.opt.img_size

        # Initialize
        set_logging()
        self.device = select_device(self.opt.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check img_size
        if self.half:
            self.model.half()  # to FP16

        # Second-stage classifier
        self.classify = False
        if self.classify:
            self.modelc = load_classifier(name='resnet101', n=2)  # initialize
            self.modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=self.device)['model']).to(
                self.device).eval()

        # read names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]
        self.x_center = 0.0
        self.y_center = 0.0

    def detect(self, source):
        xy_coords = []
        if type(source) != list:
            raise TypeError('source must be a list which contain  pictures read by cv2')

        # Set Dataloader
        dataset = MyLoadImages(source, img_size=self.imgsz, stride=self.stride)
    
        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(
                next(self.model.parameters())))  # run once
        result = []
 
        for img, im0s in dataset:
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            pred = self.model(img, augment=self.opt.augment)[0]
            # Apply NMS
            pred = non_max_suppression(pred, self.opt.conf_thres, self.opt.iou_thres, classes=self.opt.classes,
                                       agnostic=self.opt.agnostic_nms)

            # Apply Classifier
            if self.classify:
                pred = apply_classifier(pred, self.modelc, img, im0s)

           
            # Process detections
            det = pred[0] 
            im0 = im0s.copy() 
            result_txt = []
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                '''
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string
                '''
                # Write results

                for *xyxy, conf, cls in reversed(det):

                    line = (int(cls.item()), [int(_.item()) for _ in xyxy], conf.item())  # label format
                    result_txt.append(line)
                    self.x_center = (xyxy[0].item() + xyxy[2].item()) / 2.0  # X coordinate
                    self.y_center = (xyxy[1].item() + xyxy[3].item()) / 2.0  # Y coordinate

                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=3)
                    xy_coords.append([self.x_center,self.y_center])
            result.append((im0, result_txt)) 
        return result, self.names, xy_coords

