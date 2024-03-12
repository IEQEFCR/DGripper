import rospy
import cv2 as cv
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def cv_image_callback(msg):
    # print('Received an image!')
    global cv_image
    #rotate image
    cv_image = cv.rotate(bridge.imgmsg_to_cv2(msg, "bgr8"), cv.ROTATE_90_CLOCKWISE)

if __name__ == '__main__':
    rospy.init_node('yolo')
    bridge = CvBridge()
    cv_image = None
    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, cv_image_callback)
    
    # Load YOLO
    model = YOLO('best.pt')

    cv.namedWindow('yolo', cv.WINDOW_NORMAL)
    while not rospy.is_shutdown():
        if cv_image is not None:
            
            # Detect
            result = model.predict(cv_image)[0].cpu()
            if result.boxes :
                print(result)
                for i in result.boxes:
                    box = i.xywh[0]
                    cv_image =cv.rectangle(cv_image, (int(box[0]-box[2]/2), int(box[1]-box[3]/2)), (int(box[0]+box[2]/2), int(box[1]+box[3]/2)), (0, 255, 0), 2)
                    class_name = model.names[int(i.cls)]
                    cv.putText(cv_image, class_name, (int(box[0]-box[2]/2), int(box[1]-box[3]/2)), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            # print(result)
        cv.imshow('yolo', cv_image)
        cv.waitKey(1)
    cv.destroyAllWindows()