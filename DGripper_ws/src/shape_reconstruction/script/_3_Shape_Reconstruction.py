import cv2
import yaml
from sensor import Sensor
from visualizer import Visualizer

if __name__ == '__main__':
    f = open("shape_config.yaml", 'r+', encoding='utf-8')
    cfg = yaml.load(f, Loader=yaml.FullLoader)
    sensor = Sensor(cfg)
    visualizer = Visualizer(sensor.points)
    depth_map_video = cv2.VideoWriter('depth_map.mp4', cv2.VideoWriter_fourcc(*'MJPG'), 20, (460,345))
    while sensor.cap.isOpened():
        img = sensor.get_rectify_crop_image()
        img_GRAY = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow('RawImage_GRAY', img_GRAY)
        height_map = sensor.raw_image_2_height_map(img_GRAY)
        depth_map = sensor.height_map_2_depth_map(height_map)
        cv2.imshow('DepthMap', depth_map)
        height_map = sensor.expand_image(height_map)
        depth_map_video.write(cv2.cvtColor(depth_map, cv2.COLOR_GRAY2BGR))
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        if not visualizer.vis.poll_events():
            break
        else:
            points, gradients = sensor.height_map_2_point_cloud_gradients(
                height_map)
            visualizer.update(points, gradients)
