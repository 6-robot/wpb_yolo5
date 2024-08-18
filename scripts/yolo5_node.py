#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
import os
import torch
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from wpb_yolo5.msg import BBox2D

start = True

# 彩色图像回调函数
def cbImage(msg):
    global start
    if start == False:
        return

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    global model
    # Inference
    results = model(cv_image, size=640)  # includes NMS
    results.print()
    bboxs = results.pandas().xyxy[0].values
    bbox2d_msg = BBox2D()
    for bbox in bboxs:
        # rospy.logwarn("识别物品 %s (%.2f , %.2f)" , bbox[-1],bbox[0],bbox[1])
        bbox2d_msg.name.append(bbox[-1])
        bbox2d_msg.probability.append(bbox[4])
        bbox2d_msg.left.append(bbox[0])
        bbox2d_msg.top.append(bbox[1])
        bbox2d_msg.right.append(bbox[2])
        bbox2d_msg.bottom.append(bbox[3])
    global bbox2d_pub
    bbox2d_pub.publish(bbox2d_msg)
    
    # 弹出窗口显示图片
    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(1)

def cbCommand(msg):
    global start
    rospy.logwarn('Yolo接受到' + msg.data)
    if msg.data == 'yolo start':
        rospy.logwarn('Yolo识别开启...')
        start = True
    if msg.data == 'yolo stop':
        start = False
        rospy.logwarn('Yolo识别停止！')

# 主函数
if __name__ == "__main__":
    rospy.init_node("yolo5_node")
    weights_path = os.environ['HOME'] + '/yolo5_weights/best.pt' #'/home/robot/Work/yolov5/runs/train/exp/weights/best.pt'
    yolov5_path = os.environ['HOME'] + '/Work/yolov5/'
    rospy.logwarn('Weights : ' + weights_path)
    # model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path)
    # model = torch.hub.load('/home/robot/.cache/torch/hub/ultralytics_yolov5_master/', 'custom', path=weights_path, source='local')
    model = torch.hub.load(yolov5_path, 'custom', path=weights_path, source='local')
    # 订阅机器人视觉传感器Kinect2的图像话题
    # image_sub = rospy.Subscriber("/kinect2/qhd/image_color_rect",Image,cbImage,queue_size=10)
    # image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,cbImage,queue_size=10)
    image_sub = rospy.Subscriber("/yolo_cmd",String,cbCommand,queue_size=1)
    # image_sub = rospy.Subscriber("/camera/color/image_raw",Image,cbImage,queue_size=10)

    image_sub = rospy.Subscriber("/yolo/image_predict",Image,cbImage,queue_size=10)
    
    bbox2d_pub = rospy.Publisher("/yolo_bbox_2d", BBox2D, queue_size=10)
    rospy.spin()