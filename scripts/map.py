#!/usr/bin/env python
import rospy
import rospkg
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose


class Map:
    def __init__(self):
        rospy.init_node("map")
        self.pose = None
        self.scan = None
        self.pose2d = None
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scanCallback)
        self.pose_sub = rospy.Subscriber("/localization/pose", PoseStamped, self.poseCallback)

        rospack = rospkg.RosPack()
        path_maze = rospack.get_path('ras_maze_map')
        path_map = rospack.get_path('ras_map')
        self.map_file = path_map + "/map_image/lab_maze_2017.png"
        self.maze_file = path_maze + "/maps/lab_maze_2017.txt"

        self.map = cv2.imread(self.map_file, 0)
        self.cell_size = 0.01
        self.b = int(0.5 / self.cell_size)

        maze_file = open(self.maze_file)
        lines = np.array([map(float, line.split(' ')) for line in maze_file])
        max_x = max(max(lines[:, 0]), max(lines[:, 2]))  # max x
        max_y = max(max(lines[:, 1]), max(lines[:, 3]))  # max y
        # min_x = min(min(lines[:,0]), min(lines[:, 2])) # min x
        # min_y = min(min(lines[:,1]), min(lines[:, 3])) # min y
        self.w = int(np.ceil(max_x / self.cell_size))
        self.h = int(np.ceil(max_y / self.cell_size))    
        if self.map is None:
            lines_img = (lines / self.cell_size).astype(int)
            self.map = np.zeros((self.w, self.h))
            for ln in lines_img:
                cv2.line(self.map, (ln[0], ln[1]), (ln[2], ln[3]), 255, 1)
            cv2.imwrite(self.map_file, self.map)
        cv2.imshow('image', self.map)
        cv2.waitKey(1)

    def update(self):
        if self.scan is None or self.pose2d is None:
            return
        pxy = np.array([self.scan[0] * np.cos(self.scan[1] + self.pose2d[2]),
                        self.scan[0] * np.sin(self.scan[1] + self.pose2d[2])]) + self.pose2d[:2, np.newaxis]
        p = (pxy / self.cell_size).astype(int)
        p = p[:, np.logical_and(np.logical_and(p[0]>0, p[1]>0), np.logical_and(p[0]<self.w, p[1]<self.h))] # this might not be correct
        pose_cell = (int(self.pose2d[0]/self.cell_size), int(self.pose2d[1]/self.cell_size))
        im = cv2.imread(self.map_file, 0)
        cv2.line(im,
                 (pose_cell[0] + 5, pose_cell[1] - 5),
                 (pose_cell[0] - 5, pose_cell[1] + 5), 255, 1)
        cv2.line(im,
                 (pose_cell[0] + 5, pose_cell[1] + 5),
                 (pose_cell[0] - 5, pose_cell[1] - 5), 255, 1)
        im = im.astype(int)
        im[p[1], p[0]] += 10
        (h, w) = self.map.shape
        im[((pose_cell[1] - self.b) if (pose_cell[1] - self.b >= 0) else 0):
((pose_cell[1] + self.b + 1) if (pose_cell[1] + self.b < w) else w),
((pose_cell[0] - self.b) if (pose_cell[0] - self.b >= 0) else 0):
((pose_cell[0] + self.b + 1) if (pose_cell[0] + self.b < h) else h)] -= 1
        im[im > 255] = 255
        im[im < 0] = 0
        self.map = im.astype(np.uint8)
        cv2.imwrite(self.map_file, self.map)

    def scanCallback(self, msg):
        ranges = list(msg.ranges)
        bearings = [i for i in range( int( round((msg.angle_max-msg.angle_min)/msg.angle_increment) ) + 1)]
        self.scan = np.array([ranges, bearings])
        self.scan = self.scan[:, np.isfinite(ranges)]
        print "Received scan"

    def poseCallback(self, msg):
        self.pose = msg.pose
        self.pose2d = np.array([msg.pose.position.x, msg.pose.position.y, 2*np.arcsin(msg.pose.orientation.z)])
        print "Received pose"


if __name__ == '__main__':
    m = Map()

    rate = rospy.Rate(10)
    try:
        while True:
            m.update()

            cv2.imshow('image', cv2.resize(m.map, None, fx=2, fy=2))
            cv2.waitKey(1)
            rate.sleep()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
