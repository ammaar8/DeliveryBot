#!/usr/bin/env python3

from __future__ import print_function

import roslib
roslib.load_manifest('deliverybot_vision')
import sys
import rospy
import easyocr
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher('door_number_image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.ocr_reader = easyocr.Reader(['en'], gpu=True)
        self.image_sub = rospy.Subscriber('/dbot/kinect/color/image_raw', Image, self.callback)


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        res = self.run_ocr(cv_image)

        cv2.waitKey(5)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, 'bgr8'))
        except CvBridgeError as e:
            print(e)
    
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
                org=(contours[0][0][0], contours[1][0][1]+150),
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