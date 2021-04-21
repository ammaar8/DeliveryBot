#!/usr/bin/env python3

from __future__ import print_function

import roslib
roslib.load_manifest('deliverybot_vision')
import sys
import os
import rospy
import easyocr
from cv2 import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher('door_number_image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.ocr_reader = easyocr.Reader(['en'], gpu=True)
        self.image_sub = rospy.Subscriber('/dbot/upper_camera/upper_camera_image', Image, self.callback)
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.yolo_weights = os.path.join(dir_path, "weights_configs/yolo-obj.weights")
        self.yolo_config = os.path.join(dir_path, "weights_configs/yolo-obj.cfg")
        self.yolo = cv2.dnn.readNet(self.yolo_weights, self.yolo_config)
        with open(os.path.join(dir_path, "weights_configs/obj.names"), "rt") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.layer_names = self.yolo.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.yolo.getUnconnectedOutLayers()]

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        res = self.run_ocr(cv_image)
        res = self.run_detection(res)

        cv2.waitKey(10)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, 'bgr8'))
        except CvBridgeError as e:
            print(e)
    
    def run_detection(self, image):
        try:
            height, width, _ = image.shape
            blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            self.yolo.setInput(blob)
            outputs = self.yolo.forward(self.output_layers)
            class_ids = []
            confidences = []
            boxes = []
            for output in outputs:
                for detection in output:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5: #Ideally should work with higher tolerance
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)

                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)

                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
                        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

            img = image.copy()
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]
                    label = str(self.classes[class_ids[i]])
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0,255,0), 3)
                    cv2.putText(img, label, (x, y + 100), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 3)
            return img
        except:
            return image

    def run_ocr(self, image):
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            result: list = self.ocr_reader.readtext(gray)
            door_number = result[0][1]
            location = result[0][0]
            contours = np.array(location).reshape((-1, 1, 2)).astype(np.int32)
            font = cv2.FONT_HERSHEY_COMPLEX
            res = cv2.putText(
                image,
                text=door_number,
                org=(contours[0][0][0], contours[1][0][1]+50),
                fontFace=font,
                fontScale=1,
                color=(0,255,0),
                thickness=2,
                lineType=cv2.LINE_AA
                )
            res = cv2.rectangle(
                image,
                tuple(contours[0][0]),
                tuple([2][0]),
                (0,255,0),
                3
                )
            return  res
        except:
            return image


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting Down')
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)