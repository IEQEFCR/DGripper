import rospy
import cv2 as cv

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def cv_image_callback(msg):
    print('Received an image!')
    global cv_image
    #rotate image
    cv_image = cv.rotate(bridge.imgmsg_to_cv2(msg, "bgr8"), cv.ROTATE_90_CLOCKWISE)

if __name__ == '__main__':
    rospy.init_node('yolo')
    bridge = CvBridge()
    cv_image = None
    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, cv_image_callback)
    cv.namedWindow('yolo', cv.WINDOW_NORMAL)
    while not rospy.is_shutdown():
        if cv_image is not None:
            cv.imshow('yolo', cv_image)
        cv.waitKey(1)
    cv.destroyAllWindows()