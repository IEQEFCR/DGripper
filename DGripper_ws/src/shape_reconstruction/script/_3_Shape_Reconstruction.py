import os
import cv2
import numpy as np
import yaml
from sensor import Sensor
from visualizer import Visualizer

def tactile_img(sensor):
    img = sensor.get_rectify_crop_image()
    print(img.shape)
    img_GRAY = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('RawImage_GRAY', img_GRAY)
    height_map = sensor.raw_image_2_height_map(img_GRAY)
    depth_map = sensor.height_map_2_depth_map(height_map)
    cv2.imshow('DepthMap', depth_map)
    height_map = sensor.expand_image(height_map)
    # depth_map_video.write(cv2.cvtColor(depth_map, cv2.COLOR_GRAY2BGR))
    return depth_map

if __name__ == '__main__':

    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    #set camera to grayscale
    #change to the directory of the script
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    f = open("shape_config.yaml", 'r+', encoding='utf-8')
    cfg = yaml.load(f, Loader=yaml.FullLoader)
    cfg['camera_setting']['camera_channel'] =2
    sensor1 = Sensor(cfg)
    cfg['camera_setting']['camera_channel'] =4
    sensor2 = Sensor(cfg)
    index = 0
    # visualizer = Visualizer(sensor.points)
    depth_map_video = cv2.VideoWriter('depth_map.mp4', cv2.VideoWriter_fourcc(*'MJPG'), 20, (460,345))
    while sensor1.cap.isOpened() and sensor2.cap.isOpened():
        t1 = tactile_img(sensor1)
        t2 = tactile_img(sensor2)
        show_img = np.zeros((345,460,3), np.uint8)
        camera_img = camera.read()[1]
        #cvt to grayscale
        camera_img_GRAY = cv2.cvtColor(camera_img, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Camera', camera_img_GRAY)
        camera_img_GRAY = cv2.resize(camera_img_GRAY, (460,345))
        #show to 3channel
        # show_img = cv2.cvtColor(t1, cv2.COLOR_GRAY2BGR)
        show_img[:,:,0] = t1.astype(np.uint8)*2
        show_img[:,:,2] = t2.astype(np.uint8)*2
        show_img[:,:,1] = camera_img_GRAY//2
        # show_img = cv2.cvtColor(t2, cv2.COLOR_GRAY2BGR)
        #
        #print type of show_img
        # print(show_img)
        cv2.imshow('Merged', show_img)
        key = cv2.waitKey(1)
        if key == ord('s'):
            cv2.imwrite(str(index)+'_merged.png', show_img)
            index += 1
        if key == ord('q'):
            break
        # if not visualizer.vis.poll_events():
        #     break
        # else:
        #     points, gradients = sensor.height_map_2_point_cloud_gradients(
        #         height_map)
        #     visualizer.update(points, gradients)
