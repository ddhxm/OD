#! /usr/bin/env python

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import rospy
import time

def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z","intensity"), skip_nans=True)
    time.sleep(1)
    #print(list(gen))
    print type(gen)
    points_list = []
    for p in gen:
      #print " x : %.3f  y: %.3f  z: %.3f intensity:%.3f" %(p[0],p[1],p[2],p[3])
      points_list.append([p[0], p[1], p[2], p[3]])
    print("points:",points_list)
    print("points's length:",len(points_list))

def main():
    rospy.init_node('hesai_lidar', anonymous=True)
    rospy.Subscriber('/hesai/pandar', PointCloud2, callback_pointcloud)
    rospy.spin()

if __name__ == "__main__":
    main()

