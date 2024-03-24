#!/usr/bin/env python
#coding=utf-8
#import roslib;roslib.load_manifest('beginner_tutorials')
import cv2
import detectros
cap=cv2.VideoCapture(0)
a=detectros.detectapi(weights='/home/rykj/catkin_ws/src/beginner_tutorials/scripts/runs/train5/exp/weights/best.pt')
while True:
    rec,img = cap.read()
    result,names = a.detect([img])
    img=result[0][0] #第一张图片的处理结果图片
    cv2.imshow("realepo",img)
    if cv2.waitKey(1)==27:
        break
