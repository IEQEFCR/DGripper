import os
import cv2
import numpy as np
import yaml
from sensor import Sensor
from visualizer import Visualizer
import rospy
from std_msgs.msg import Float32MultiArray

def tactile_img(sensor,index=1):
    img = sensor.get_rectify_crop_image()
    img_GRAY = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = np.zeros_like(img_GRAY)
    mask[img_GRAY>180] = 255
    mask[img_GRAY<80] = 255
    # mask self.mask或运算
    mask = mask | sensor.mask

    kernel = np.ones((5,5),np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=1)
    img_GRAY = cv2.inpaint(img_GRAY, mask, 7, cv2.INPAINT_TELEA)

    # cv2.imshow('RawImage_GRAY', img_GRAY)
    height_map = sensor.raw_image_2_height_map(img_GRAY)
    depth_map = sensor.height_map_2_depth_map(height_map)

    if index == 2:
        depth_map = cv2.flip(depth_map, 1)
        img = cv2.flip(img, 1)
    #cvtColor depth_map to BGR
    img = cv2.cvtColor(depth_map.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    #除去depth_map中0值以外的排名25%的值
    if np.max(depth_map) > 0:
        rk25 = np.percentile(depth_map[depth_map>0], 20)
    else :
        rk25 = 0
    main_body = depth_map.copy()
    main_body[depth_map < rk25] = 0

    depth_sum = np.sum(main_body)/10000
    depth_sum = int(depth_sum)
    dx, dz = 0, 0
    #findContours in main_body
    contours, _ = cv2.findContours(main_body.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #find the max contour
    # main_body = cv2.cvtColor(main_body.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    if len(contours) > 0:
        max_contour = max(contours, key=cv2.contourArea)
        #if the area of the max contour is too small, ignore it
        if cv2.contourArea(max_contour) > 800:
            #draw the max contour
            # cv2.drawContours(img, [max_contour], -1, 255, -1)
            #mask_contour的重心
            M = cv2.moments(max_contour)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            sensor.center_point.append((cx, cy))
            dx = img.shape[0]//2-cy
            dz = cx - img.shape[1]//2
            if len(sensor.center_point) > 5:
                sensor.center_point.pop(0)
            avg_center = np.mean(sensor.center_point, axis=0)
            cv2.circle(img, (int(avg_center[0]), int(avg_center[1])), 5, (255, 0, 0), -1)
            #draw the center of the max contour red
            # cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
            #轴线提取
            #轮廓的最小外接矩形
            rect = cv2.minAreaRect(max_contour)
            #矩形的四个顶点
            box = cv2.boxPoints(rect)
            #矩形的长轴，画出
            dis1 = np.linalg.norm(box[0]-box[1])
            dis2 = np.linalg.norm(box[1]-box[2])
            if dis1 < dis2:
                mid1 = (box[0]+box[1])/2
                mid2 = (box[2]+box[3])/2
            else:
                mid1 = (box[1]+box[2])/2
                mid2 = (box[3]+box[0])/2
            sensor.axis_queue.append((mid1, mid2))
            if len(sensor.axis_queue) > 5:
                sensor.axis_queue.pop(0)
            avg_axis = np.mean(sensor.axis_queue, axis=0)
            cv2.line(img, (int(avg_axis[0][0]), int(avg_axis[0][1])), (int(avg_axis[1][0]), int(avg_axis[1][1])), (0, 255, 0), 2)
            # cv2.line(img, (int(mid1[0]), int(mid1[1])), (int(mid2[0]), int(mid2[1])), (0, 255, 0), 2)

    # mask_img = np.hstack((mask, img,main_body))
    #put text depth_sum
    cv2.putText(img, 'depth_sum: '+str(depth_sum), (200, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, 'dx: '+str(dx), (200, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, 'dz: '+str(dz), (200, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    # cv2.imshow('raw'+str(index), img)
    height_map = sensor.expand_image(height_map)

    # depth_map_video.write(cv2.cvtColor(depth_map, cv2.COLOR_GRAY2BGR))
    #depth_map 转 伪彩色图
    # depth_map = depth_map*1.7
    # depth_map = depth_map.astype(np.uint8)
    # depth_map = cv2.applyColorMap(depth_map, cv2.COLORMAP_MAGMA)
    return img,dx,dz,depth_sum

if __name__ == '__main__':

    # camera = cv2.VideoCapture(0)
    # camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # ROS pulisher
    rospy.init_node('tactile_node', anonymous=True)
    rod_pub = rospy.Publisher("/rod_roll", Float32MultiArray, queue_size=10)

    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    f = open("shape_config.yaml", 'r+', encoding='utf-8')
    cfg = yaml.load(f, Loader=yaml.FullLoader)
    cfg['camera_setting']['camera_channel'] =2
    sensor1 = Sensor(cfg)
    cfg['camera_setting']['camera_channel'] =4
    sensor2 = Sensor(cfg)

    index = 0
    # visualizer = Visualizer(sensor.points)
    # depth_map_video = cv2.VideoWriter('depth_map.mp4', cv2.VideoWriter_fourcc(*'MJPG'), 20, (460,345))
    video_path ='./video/'
    if not os.path.exists(video_path):
        os.makedirs(video_path)
    while os.path.exists(video_path+str(index)+'_demo.mp4'):
        index += 1
    video = cv2.VideoWriter(video_path+str(index)+'_demo.mp4', cv2.VideoWriter_fourcc(*'MJPG'), 25, (460,690)) 
    
    start_video = False

    while sensor1.cap.isOpened() and sensor2.cap.isOpened():
        t1, dx1, dz1, depth_sum1 = tactile_img(sensor1,1)
        t2 ,dx2, dz2, depth_sum2 = tactile_img(sensor2,2)

        # publish info get from tactile_img
        msg_data = [2, dz1, dx1, dz2, dx2, depth_sum1, depth_sum2]
        pub_msg = Float32MultiArray(data=msg_data)
        rod_pub.publish(pub_msg)

        # t1 =np.hstack((t1,r1))
        # t2 =np.hstack((t2,r2))
        # t1 = np.zeros_like(t2)
        #h stack t1 and t2
        show_img = np.vstack((t1, t2))
        #show_img 上下中心画一条线
        cv2.line(show_img, (0,345), (460,345), (255,255,255), 2)
        #put text "Left" and "Right"
        cv2.putText(show_img, 'Left', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(show_img, 'Right', (10, 365), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow('Merged', show_img)
        if start_video:
            video.write(show_img)
        key = cv2.waitKey(1)
        if key == ord('s'):
            if start_video:
                print('Stop recording')
                video.release()
                index += 1
                video = cv2.VideoWriter(video_path+str(index)+'_demo.mp4', cv2.VideoWriter_fourcc(*'MJPG'), 25, (460,690))
            else:
                print('Start recording')
            start_video = not start_video
        if key == ord('q'):
            break
        # if not visualizer.vis.poll_events():
        #     break
        # else:
        #     points, gradients = sensor.height_map_2_point_cloud_gradients(
        #         height_map)
        #     visualizer.update(points, gradients)
