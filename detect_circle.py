import cv2 as cv
import numpy as np
import yaml
import rospy
import tf
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import cvui
import std_msgs.msg

class DetectCircle:
    def callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        gray = cv.cvtColor(self.image, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 5)
        rows = gray.shape[0]
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
                                  param1=100, param2=30,
                                  minRadius=1, maxRadius=30)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                self.circle = center
                radius = i[2]
                if radius < 20:
                    continue
                cv.circle(self.image, center, radius, (0, 255, 0), 3)
        self.circle_pub.publish(self.bridge.cv2_to_imgmsg(self.image, 'bgr8'))
        # self.record_botton.draw(self.image)
        #use cvui to draw a button to control the recording

    def __init__(self):
        rospy.init_node('detect_circle')
        self.rate = rospy.Rate(25)
        #往teleop pub 类型为string
        self.teleop_pub = rospy.Publisher('teleop', std_msgs.msg.String, queue_size=1)
        camera_topic = 'usb_cam/image_raw'
        self.bridge = CvBridge()
        self.circle_pub = rospy.Publisher('circle', Image, queue_size=1)
        self.sub = rospy.Subscriber(camera_topic, Image, self.callback)
        self.record_list = ["Recording", "Stop"]
        self.record = 1
        self.video_index = 0
        #mp4 video writer
        # self.video_writer = cv.VideoWriter(str(self.video_index) + '.mp4', cv.VideoWriter_fourcc(*'mp4v'), 10, (640, 480))


    def run(self):
        window_name = 'detected circles'
        cv.namedWindow(window_name)
        cvui.init(window_name)

        while not rospy.is_shutdown():
            if self.image is None:
                continue
            #在图片右边加一排空白，用来放按钮
            rotate_img = np.rot90(self.image)
            show_img = np.zeros((rotate_img.shape[0], rotate_img.shape[1] + 100, 3), dtype=np.uint8)
            show_img = 255 - show_img
            show_img[:rotate_img.shape[0], :rotate_img.shape[1]] = rotate_img
            if cvui.button(show_img, rotate_img.shape[1] , 50, 100, 50, self.record_list[self.record]):
                self.record = 1 - self.record
                if self.record == 0:
                    print('start recording')
                    self.video_index += 1
                    self.video_writer = cv.VideoWriter(str(self.video_index) + '.mp4', cv.VideoWriter_fourcc(*'mp4v'), 25, (640, 480))
                else:
                    print('stop recording')
                    print('save video to ' + str(self.video_index) + '.mp4')
                    self.video_writer.release()
            if cvui.button(show_img, rotate_img.shape[1] , 0, 100, 50, "Esc"):
                os._exit(0)
            if cvui.button(show_img, rotate_img.shape[1] , 100, 100, 50, "-90"):
                self.teleop_pub.publish(std_msgs.msg.String('a'))
            self.rate.sleep()
            if self.record == 0:
                self.video_writer.write(self.image)
            cvui.update()
            cv.imshow(window_name, show_img)
            key = cv.waitKey(1)
            if key!=-1:
                self.teleop_pub.publish(std_msgs.msg.String(chr(key)))
            if key == 27 or key == ord('q') or key == ord('Q') :
                cv.destroyAllWindows()
                os._exit(0)

if __name__ == '__main__':
    dc = DetectCircle()
    dc.run()
