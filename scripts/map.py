#!/usr/bin/env python
import rospy
import rospkg
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose

def drawMap(im, pose_cell):
    im = im.astype(np.uint8)
    cv2.line(im,
(pose_cell[0] + 5, pose_cell[1] - 5),
(pose_cell[0] - 5, pose_cell[1] + 5), 255, 1)
    cv2.line(im,
(pose_cell[0] + 5, pose_cell[1] + 5),
(pose_cell[0] - 5, pose_cell[1] - 5), 255, 1)
    cv2.imshow('image', cv2.resize(im, None, fx=2, fy=2))
    cv2.waitKey(1)

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
            self.map = np.zeros((self.h, self.w))
            for ln in lines_img:
                cv2.line(self.map, (ln[0], ln[1]), (ln[2], ln[3]), 255, 1)
            cv2.imwrite(self.map_file, self.map)
        cv2.imshow('image', self.map)
        cv2.waitKey(1)

    def update(self):
        if self.scan is None or self.pose2d is None:
            return

        pose_cell = np.array([int(self.pose2d[0]/self.cell_size), int(self.pose2d[1]/self.cell_size)])
        if pose_cell[0] < 0 or pose_cell[1] < 0 or pose_cell[0] > self.w or pose_cell[1] > self.h:
            return

        pxy = np.array([self.scan[0] * np.cos(self.scan[1] + self.pose2d[2]),
                        self.scan[0] * np.sin(self.scan[1] + self.pose2d[2])]) + self.pose2d[:2, np.newaxis]
        p = (pxy / self.cell_size).astype(int)
        p[0, p[0, :]<0]=0
        p[0, p[0, :]>=self.w]=self.w-1
        p[1, p[1, :]<0]=0
        p[1, p[1, :]>=self.h]=self.h-1

        im = cv2.imread(self.map_file, 0)
        im = im.astype(int)
        im[p[1], p[0]] += 10

        for i in range(p.shape[1]):
            angle = np.arctan2(p[1, i]-pose_cell[1], p[0, i]-pose_cell[0])
            r = int(np.sqrt((p[0,i]-pose_cell[0])*(p[0,i]-pose_cell[0]) + (p[1,i]-pose_cell[1])*(p[1,i]-pose_cell[1])))
            c = np.cos(angle)
            s = np.sin(angle)
            x = (pose_cell[0] + np.arange(1, r-1, 1)*c).astype(int)
            y = (pose_cell[1] + np.arange(1, r-1, 1)*s).astype(int)
            im[ y, x ] -= 4

        im[im > 255] = 255
        im[im < 0] = 0
        im[0, :] = 255
        im[-1, :] = 255
        im[:, 0] = 255
        im[:, -1] = 255

        self.map = im.astype(np.uint8)
        cv2.imwrite(self.map_file, self.map)
        drawMap(im, pose_cell)
        

    def scanCallback(self, msg):
        ranges = list(msg.ranges)
        bearings = [i*np.pi/180 for i in range( int( round((msg.angle_max-msg.angle_min)/msg.angle_increment) ) + 1)]
        self.scan = np.array([ranges, bearings])
        self.scan = self.scan[:, np.isfinite(ranges)]

    def poseCallback(self, msg):
        self.pose = msg.pose
        self.pose2d = np.array([msg.pose.position.x, msg.pose.position.y, 2*np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)])


if __name__ == '__main__':
    m = Map()

    rate = rospy.Rate(10)

    try:
        while True:
            m.update()
            rate.sleep()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        pass
