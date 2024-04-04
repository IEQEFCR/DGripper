// include ros header
#include <ros/package.h>
#include <ros/ros.h>
// include image msg
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
// cv bridge
#include <cv_bridge/cv_bridge.h>
// include multi-array 32
#include <std_msgs/Float32MultiArray.h>
#include <list>
#include <queue>
#include <vector>
// c++中类似python中path join的函数
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

#define time_step 0.05
#define history_time 2
#define feature_num 20
#define pixmm 0.051015        // 每个像素对应的物理距离
#define depth_threshold 0.01  // 深度阈值,以mm为单位
#define point_distance 8      // 特征点之间的像素距离

class tactile_point {  // 触觉点类
   public:
    double x, y, z;  // 在物理坐标系下的坐标
    double depth;    // 触觉的深度
    int u, v;        // 在像素坐标系下的坐标
    tactile_point(double x, double y, double z, double depth, int u, int v) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->depth = depth;
        this->u = u;
        this->v = v;
    }
    tactile_point() {}
};

class feature {  // 特征点类
   public:
    list<tactile_point> P;     // 特征点历史队列
    tactile_point last_point;  // 最新的点

    int type, count;  // 0 for point cloud,1 for line, 2 for keypoint

    feature(tactile_point p) {
        // list P add p
        this->P.push_back(p);
        this->last_point = p;
        count = 1;
    }

    void add_point(tactile_point p) {
        this->P.push_back(p);
        if (this->P.size() > feature_num)
            this->P.pop_front();
        count = this->P.size();
        this->last_point = p;
    }
    // 倒数第n个元素
    tactile_point get_back_point(int n) {
        auto it = P.end();
        for (int i = 0; i < n - 1; i++)
            it--;
        return *it;
    }
};

class gripper_state {  // 定义夹爪状态类型
   public:
    double left_finger_position, right_finger_position;  // 两指表面移动距离
    double finger_distance;                              // 两指表面距离
    double left_roller_position, right_roller_position;  // 两指滚轮位置
    gripper_state() {
        left_finger_position = 0;
        right_finger_position = 0;
        finger_distance = 0;
        left_roller_position = 0;
        right_roller_position = 0;
    }
    gripper_state(double lfp, double rfp, double fd, double lrp, double rrp) {
        left_finger_position = lfp;
        right_finger_position = rfp;
        finger_distance = fd;
        left_roller_position = lrp;
        right_roller_position = rrp;
    }
};

class angle_tracking {
   public:
    // 定义夹爪状态和触觉高度图
    gripper_state gs;
    cv::Mat tactile_heighmap[2];
    cv_bridge::CvImagePtr cv_ptr;
    vector<double> angle;            // 角度序列
    list<feature> feature_queue[2];  // 0 for left,1 for right
    // 订阅夹爪状态和触觉高度图
    void gripper_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        gs = gripper_state(msg->data[0], msg->data[1], msg->data[2],
                           msg->data[3], msg->data[4]);
    }

    void tactile_heighmap_callback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            cv_ptr =  // 数据类型为float32
                cv_bridge::toCvCopy(msg,
                                    sensor_msgs::image_encodings::TYPE_64FC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // 上半部分为左指触觉高度图,下半部分为右指触觉高度图
        tactile_heighmap[0] = cv_ptr->image(
            cv::Rect(0, 0, cv_ptr->image.cols, cv_ptr->image.rows / 2));
        tactile_heighmap[1] =
            cv_ptr->image(cv::Rect(0, cv_ptr->image.rows / 2,
                                   cv_ptr->image.cols, cv_ptr->image.rows / 2));
    }

    angle_tracking() {
        // ros node init
        // ros::NodeHandle nh;
        // gripper_sub = nh.subscribe("/motors_pos_real", 1,  // 订阅夹爪状态
        //                            &angle_tracking::gripper_callback, this);
        // tactile_heighmap_sub =
        //     nh.subscribe("/height_map", 1,  // 订阅触觉高度图
        //                  &angle_tracking::tactile_heighmap_callback, this);
    }

    void update(int side, double dz) {  // side 0 is left ,side 1 is right
        // 遍历链表
        for (auto it = feature_queue[side].begin();
             it != feature_queue[side].end();) {
            feature* f = &(*it);

            tactile_point p = f->last_point;
            p.u = p.u + (int)(dz / pixmm);
            std::cout << "dz: " << (int)(dz / pixmm) << std::endl;
            if (p.u < 0 || p.u >= tactile_heighmap[side].cols) {
                auto it_temp = it;
                it++;
                feature_queue[side].erase(it_temp);
                continue;
            }
            p.depth = tactile_heighmap[side].at<double>(p.v, p.u);
            if (p.depth < depth_threshold) {
                auto it_temp = it;
                it++;
                feature_queue[side].erase(it_temp);
                continue;
            }  // delete feature
            p.z = p.z + dz;
            p.y = p.depth + gs.finger_distance / 2;
            p.y = (side == 0 ? p.y : -p.y);
            f->add_point(p);
            it++;
        }
    }

    void pose_estimation() {
        if (feature_queue[0].empty() && feature_queue[1].empty()) {  // 初始状态
            angle.push_back(0);
            return;
        }
        // 有一边没有特征
        if (feature_queue[0].empty() || feature_queue[1].empty()) {
            angle.push_back(angle.back());  // 保持上一次的角度
            return;
        }
        int size[2] = {feature_queue[0].size(), feature_queue[1].size()};
        int less = size[0] < size[1] ? 0 : 1;
        int size_less = size[less];
        double sum1 = 0, sum2 = 0;
        for (auto it0 = feature_queue[0].begin(),
                  it1 = feature_queue[1].begin();
             size_less != 0; it0++, it1++, size_less--) {
            feature f0 = *it0, f1 = *it1;
            int c = min(f0.count, f1.count);
            // angle的倒数第c个元素
            double angle_past = angle[angle.size() - c];
            tactile_point p0_old = f0.get_back_point(c), p0_new = f0.last_point;
            tactile_point p1_old = f1.get_back_point(c), p1_new = f1.last_point;
            double n1[2] = {p0_old.z - p1_old.z, p0_old.y - p1_old.y};
            double n2[2] = {p0_new.z - p1_new.z, p0_new.y - p1_new.y};
            // n1转向n2的角度
            double angle_now = acos((n1[0] * n2[0] + n1[1] * n2[1]) /
                                    (sqrt(n1[0] * n1[0] + n1[1] * n1[1]) *
                                     sqrt(n2[0] * n2[0] + n2[1] * n2[1])));
            // 如果n1在n2的顺时针方向
            if (n1[0] * n2[1] - n1[1] * n2[0] < 0)
                angle_now = -angle_now;
            sum1 += (angle_past + angle_now) * c;
            sum2 += c;
            // 遍历两个链表
        }
        angle.push_back(sum1 / sum2);
    }

    void add_new_feature(int side) {
        // binary_img ,threshold with depth_threshold
        cv::Mat binary_img = cv::Mat::zeros(tactile_heighmap[side].size(),
                                            CV_8UC1);  // 二值图像
        cv::threshold(tactile_heighmap[side], binary_img, depth_threshold, 255,
                      cv::THRESH_BINARY);  // 大于depth_threshold的像素点为255
        binary_img.convertTo(binary_img, CV_8UC1);

        // 消除mask中现有点区域
        for (auto it = feature_queue[side].begin();
             it != feature_queue[side].end(); it++) {
            feature f = *it;
            auto p = f.last_point;
            // print p u,v
            cv::circle(binary_img, cv::Point(p.u, p.v), point_distance,
                       cv::Scalar(0), -1);
        }
        // find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_img, contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);
        for (auto contour : contours) {
            cv::Rect rect = cv::boundingRect(contour);
            for (int i = rect.x; i < rect.x + rect.width; i += point_distance) {
                for (int j = rect.y; j < rect.y + rect.height;
                     j += point_distance) {
                    if (binary_img.at<uchar>(j, i) == 255) {
                        // i是列数,j是行数
                        double x =
                            (j - tactile_heighmap[side].rows / 2) * pixmm;
                        double y = tactile_heighmap[side].at<float>(j, i) +
                                   gs.finger_distance / 2;
                        double z =
                            (i - tactile_heighmap[side].cols / 2) * pixmm;
                        y = (side == 0 ? y : -y);
                        tactile_point p(x, y, z,
                                        tactile_heighmap[side].at<float>(j, i),
                                        i, j);
                        feature_queue[side].push_back(feature(p));
                    }
                }
            }
        }
    }

    void show_feature() {
        cv::Mat img[2] = {cv::Mat::zeros(tactile_heighmap[0].size(), CV_8UC3),
                          cv::Mat::zeros(tactile_heighmap[1].size(), CV_8UC3)};
        for (int side = 0; side < 2; side++) {
            for (auto it = feature_queue[side].begin();
                 it != feature_queue[side].end(); it++) {
                feature f = *it;
                auto p = f.last_point;
                cv::circle(img[side], cv::Point(p.u, p.v), 2,
                           cv::Scalar(0, 255, 0), -1);
            }
        }
        cv::imshow("feature0", img[0]);
        cv::imshow("feature1", img[1]);
        cv::waitKey(1);
    }

    void run() {
        ros::NodeHandle nh;
        ros::Subscriber gripper_sub;
        ros::Subscriber tactile_heighmap_sub;
        gripper_sub = nh.subscribe("/motors_pos_real", 1,  // 订阅夹爪状态
                                   &angle_tracking::gripper_callback, this);
        tactile_heighmap_sub =
            nh.subscribe("/height_map", 1,  // 订阅触觉高度图
                         &angle_tracking::tactile_heighmap_callback, this);
        ros::Rate loop_rate(1 / time_step);
        gripper_state last_gripper_state = gs, now_gripper_state;

        while (ros::ok()) {
            loop_rate.sleep();
            ros::spinOnce();         // 读取夹爪状态和触觉高度图
            now_gripper_state = gs;  // 读取当前夹爪状态
            if (tactile_heighmap[0].size().width == 0)
                continue;

            update(1,
                   now_gripper_state.left_finger_position -  // 更新左指特征
                       last_gripper_state.left_finger_position);
            update(0,
                   now_gripper_state.right_finger_position -  // 更新右指特征
                       last_gripper_state.right_finger_position);
            // pose_estimation();  // 利用目前更新完成的特征点进行位姿估计

            add_new_feature(0);  // 添加新的触觉特征
            add_new_feature(1);
            show_feature();  // 显示特征点
            last_gripper_state = now_gripper_state;
        }
    }

    //     void debug() {
    //         ros::NodeHandle nh;
    //         ros::Subscriber gripper_sub;
    //         ros::Subscriber tactile_heighmap_sub;
    //         gripper_sub = nh.subscribe("/motors_pos_real", 1,  //
    //         订阅夹爪状态
    //                                    &angle_tracking::gripper_callback,
    //                                    this);
    //         tactile_heighmap_sub =
    //             nh.subscribe("/height_map", 1,  // 订阅触觉高度图
    //                          &angle_tracking::tactile_heighmap_callback,
    //                          this);
    //         ros::Rate loop_rate(1 / time_step);
    //         gripper_state last_gripper_state = gs, now_gripper_state;
    //         while (ros::ok()) {
    //             loop_rate.sleep();
    //             ros::spinOnce();         // 读取夹爪状态和触觉高度图
    //             now_gripper_state = gs;  // 读取当前夹爪状态

    //             if (tactile_heighmap[0].size().width == 0) {
    //                 continue;
    //             }
    //             add_new_feature(0);  // 添加新的触觉特征
    //             add_new_feature(1);
    //             // print feature_list
    //             for (int side = 0; side < 2; side++) {
    //                 for (auto it = feature_queue[side].begin();
    //                      it != feature_queue[side].end(); it++) {
    //                     feature f = *it;
    //                     tactile_point p = f.last_point;
    //                     // print p u v
    //                     std::cout << p.u << " " << p.v << std::endl;
    //                 }
    //             }
    //             show_feature();  // 显示特征点
    //             cv::waitKey(1);
    //         }
    //     }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tactile_pose_tracking");
    angle_tracking at;
    at.run();
    return 0;
}

// git 切换分支的命令是 git checkout 分支名