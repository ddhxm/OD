# -*- coding: utf-8 -*-
#!/usr/bin/env python3


import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import time
import calibration_kitti
import threading
import random


def get_fov_flag(pts_rect, width,height, calib):
    """
    Args:
        pts_rect:
        img_shape:.(h,w)
        calib:

    Returns:

    """
    pts_img, pts_rect_depth = calib.rect_to_img(pts_rect)  # p0转到P2坐标系
    val_flag_1 = np.logical_and(pts_img[:, 0] >= 0, pts_img[:, 0] < width)  # (0,w)
    val_flag_2 = np.logical_and(pts_img[:, 1] >= 0, pts_img[:, 1] < height)  # (0,h)
    val_flag_merge = np.logical_and(val_flag_1, val_flag_2)
    pts_valid_flag = np.logical_and(val_flag_merge, pts_rect_depth >= 0)

    return pts_valid_flag


def get_box_point(centers, pts_img, box_xyxy):
    """
    Args:
        pts_rect:
        img_shape:
        calib:

    Returns:返回list

    """
    boxes_flag = []
    point_in_boxes = []
    box_depth_list = []
    centers = np.array(centers)[:, 0:2]
    for i in range(len(box_xyxy)):
        center = centers[i]
        box = box_xyxy[i]  # 得到两角点xy的范围
        x_min = box[0]
        x_max = box[2]
        y_min = box[1]
        y_max = box[3]

        val_flag_1 = np.logical_and(pts_img[:, 0] >= x_min, pts_img[:, 0] <= x_max)
        val_flag_2 = np.logical_and(pts_img[:, 1] >= y_min, pts_img[:, 1] <= y_max)
        box_valid_flag = np.logical_and(val_flag_1, val_flag_2)
        point_in_box = pts_img[box_valid_flag]
        point_num = len(point_in_box)
        if len(point_in_box) == 0:
            print('------------------------------------------')
            print("此box{}内无点云投影点,因此从所有点范围内选择最近的三个点".format(center))
            distance = ((pts_img[:, 0] - center[0]) ** 2 + ((pts_img[:, 1] - center[1]) ** 2)) ** 0.5
            #slect_neigh = distance.sort().values[:3]  # 选择邻近的3个点
            slect_index = np.argsort(distance)[:10]
            slect_point = pts_img[slect_index]
            print("select the nearest 3 points:", slect_point)
            print("the depth of the selected points:", slect_point[:, 2])
            box_depth = np.mean(slect_point[:, 2])
            print(box_depth)
            print("box's depth：", box_depth)
            box_depth_list.append(box_depth)
        else:
            print('------------------------------------------')
            print("box{}内含有点云投影点,点数为:{}".format(center,point_num))
            boxes_flag.append(box_valid_flag)
            point_in_boxes.append(point_in_box)
            distance = ((point_in_box[:, 0] - center[0]) ** 2 + ((point_in_box[:, 1] - center[1]) ** 2)) ** 0.5
            if point_num>10:
                #slect_neigh = np.sort(distance).values[:10]  # 选择邻近的10个点
                slect_index = np.argsort(distance)[:10]
                slect_point = point_in_box[slect_index]
                print("select the nearest 10 points:", slect_point)
                print("the depth of the selected points:", slect_point[:, 2])
                box_depth = np.mean(slect_point[:, 2])
                print("box's depth:", box_depth)
                box_depth_list.append(box_depth)
            else:
                #slect_neigh = distance.sort().values[:point_num]  # 选择邻近的10个点
                slect_index = np.argsort(distance)[:point_num]
                slect_point = point_in_box[slect_index]
                print("select the nearest {}points:{}".format(point_num, slect_point))
                print("the depth of the selected points:", slect_point[:, 2])
                box_depth = np.mean(slect_point[:, 2])
                print("box's depth:", box_depth)
                box_depth_list.append(box_depth)

    return  np.concatenate((centers,np.expand_dims(box_depth_list, 1)), axis=1)


    
class data_collect_processing():

    def __init__(self):

        rospy.init_node('hesai', anonymous=True)

        self.bridge = CvBridge()
        self.points_list = []
        self.box_xyxy=[]
        self.box_centers = []
        self.width = 0
        self.height = 0
        self.object_class = []
        self.depth_image = []

        imge = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        bboxes = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.BoundingBox_callback)
        pointcloud = rospy.Subscriber('/hesai/pandar', PointCloud2, self.callback_pointcloud)




    def callback_pointcloud(self,data):
        assert isinstance(data, PointCloud2)
        lidar = point_cloud2.read_points(data, field_names=("x", "y", "z","intensity"), skip_nans=True)  # 返回generator生成器，list(lidar)得到所有点
        time.sleep(1)

        for p in lidar:
            #print " x : %.3f  y: %.3f  z: %.3f intensity:%.3f" %(p[0],p[1],p[2],p[3])
            self.points_list.append([p[0], p[1], p[2], p[3]])
        print("points's length:", len(self.points_list))  # list形式
        box_de = calib(self.points_list, self.depth_image, self.object_class, self.width, self.height, self.box_centers, self.box_xyxy)




    def BoundingBox_callback(self, data):
        bbox = data.bounding_boxes
        xyxy = []
        centers = []
        classes = []
        for i in range(len(bbox)):
            probability = bbox[i].probability
            xmin = bbox[i].xmin
            ymin = bbox[i].ymin
            xmax = bbox[i].xmax
            ymax = bbox[i].ymax
            ids = bbox[i].id
            Class = bbox[i].Class
            center_xu = (xmax-xmin)/2 + xmin
            center_yv = (ymax-ymin)/2 + ymin
            width = xmax - xmin
            height = ymax - ymin
            xyxy.append([xmin, ymin, xmax, ymax])
            centers.append([center_xu, center_yv, width, height])
            classes.append(Class)
            #print('object:{}, box coord:{}, {}, {}, {}'.format(Class, center_xu, center_yv, width, height))
        self.box_xyxy = xyxy
        self.box_centers = centers
        #print(centers)
        self.object_class = classes

        point_test = np.random.randint(-10, 100, (16384, 3))
        box_de = calib(point_test, self.depth_image, self.object_class, self.width, self.height, self.box_centers,self.box_xyxy)


        image = cv2.cvtColor(self.depth_image, cv2.COLOR_BGR2RGB)
        for i in range(len(self.box_xyxy)):
                xmin = self.box_xyxy[i][0]
                ymin = self.box_xyxy[i][1]
                xmax = self.box_xyxy[i][2]
                ymax = self.box_xyxy[i][3]

                cv2.rectangle(image, (xmin, ymin),
                              (xmax, ymax),
                              (255, 0, 0), 1)
                cv2.putText(image, classes[i] + " " + (str(box_de[i, 0]) + " m"),
                            (xmin, ymin),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)

        cv2.imshow("test", image)
        cv2.waitKey(10)


        

    def image_callback(self, image_data):

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #print("transformed image:", self.depth_image.shape)
        self.width = image_data.width
        self.height = image_data.height
        #print("width,height:", self.width, self.height)

        box_centers = np.random.randint(0, 320, (10, 4))
        box_xyxy = np.random.randint(0, 100, (10, 4))




def calib(points, image, classes, width, height, centers, box_xyxy):
        calib_file = '000016.txt'
        cal = calibration_kitti.Calibration(calib_file)
        #print(np.array(points))

        if len(points) == 0:
            print("waiting for the points message!")
        else:
            pts_rect = cal.lidar_to_rect(np.array(points)[:, 0:3])
            pts_fov_flag = get_fov_flag(pts_rect, width, height, cal)  # 转到P0的点云点，再转到P2图像，得到前面范围内的点标志
            pts_fov = pts_rect[pts_fov_flag]  # P0坐标系下图像fov点
            print(pts_fov)
            pts_img_2d, pts_rect_depth = cal.rect_to_img(pts_fov)  # 从P0参考相机转到P2相机的像素坐标系
            pts_rect_depth = np.expand_dims(pts_rect_depth, axis=1)  # (N,1)
            pts_img_3d = np.concatenate((pts_img_2d, pts_rect_depth), axis=1)
            #print('图像P2坐标系投影点：', pts_img_2d)
            print(pts_img_3d.shape)
            if len(centers) == 0:
                print("waiting for the darknet_ros message!")
            else:
                box_with_depth = get_box_point(centers, pts_img_3d, box_xyxy)
                print("the coordinate of boxes in P2 and the depth:", box_with_depth)

                box_with_depth_rect = cal.img_to_rect(box_with_depth[:,0], box_with_depth[:, 1], box_with_depth[:, 2])
                #print("P0坐标系中：",box_with_depth_rect)
                box_with_depth_lidar = cal.rect_to_lidar(box_with_depth_rect)  # box中心depth
                print("lidar coordinate:", box_with_depth_lidar)
                return box_with_depth_lidar









if __name__ == '__main__':
    try:
        print("strat")
        detector = data_collect_processing()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Detector node terminated.")
