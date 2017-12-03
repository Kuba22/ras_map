#!/usr/bin/env python
import rospy
import rospkg
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose
from sklearn.neighbors import NearestNeighbors
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

increment_rate = 12
decrement_rate = 3
threshold = 100

def drawMap(im, pose_cell, name):
    im = im.astype(np.uint8)
    cv2.line(im,
(pose_cell[0] + 5, pose_cell[1] - 5),
(pose_cell[0] - 5, pose_cell[1] + 5), 255, 1)
    cv2.line(im,
(pose_cell[0] + 5, pose_cell[1] + 5),
(pose_cell[0] - 5, pose_cell[1] - 5), 255, 1)
    cv2.imshow(name, cv2.resize(im, None, fx=3, fy=3))
    cv2.waitKey(1)

def boundp(p, h, w):
    p[0, p[0, :]<0]=0
    p[0, p[0, :]>=w]=w-1
    p[1, p[1, :]<0]=0
    p[1, p[1, :]>=h]=h-1
    return p

def prepIm(im, p, pose_cell, b):
    im[p[1], p[0]] += increment_rate
    for i in range(p.shape[1]):
        angle = np.arctan2(p[1, i]-pose_cell[1], p[0, i]-pose_cell[0])
        r = int(np.sqrt((p[0,i]-pose_cell[0])*(p[0,i]-pose_cell[0]) + (p[1,i]-pose_cell[1])*(p[1,i]-pose_cell[1])))
        c = np.cos(angle)
        s = np.sin(angle)
        x = (pose_cell[0] + np.arange(1, r-1, 1)*c).astype(int)
        y = (pose_cell[1] + np.arange(1, r-1, 1)*s).astype(int)
        im[ y, x ] -= decrement_rate
    battery_half_size = 2
    if b is not None:
        for bt in b:
            im[ -battery_half_size+bt[1]:battery_half_size+bt[1], -battery_half_size+bt[0]:battery_half_size+bt[0] ] = 255
    im[im > 255] = 255
    im[im < 0] = 0
    im[0, :] = 255
    im[-1, :] = 255
    im[:, 0] = 255
    im[:, -1] = 255
    return im

class Map:
    def __init__(self):
        rospy.init_node("map")
        self.pose = None
        self.scan = None
        self.pose2d = None
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scanCallback)
        self.img_pub = rospy.Publisher("/map", Image, queue_size=2)
        self.pose_sub = rospy.Subscriber("/localization/pose", PoseStamped, self.poseCallback)
        self.bridge = CvBridge()
		
        rospack = rospkg.RosPack()
        path_maze = rospack.get_path('ras_maze_map')
        self.path_map = rospack.get_path('ras_map')
        self.map_file = self.path_map + "/map_image/lab_maze_2017.png"
        self.map_file_th = self.path_map + "/map_image/map.png"
        self.maze_file = path_maze + "/maps/lab_maze_2017.txt"
        #self.battery_file = rospack.get_path('detection')+"/mapFiles/battery.txt"
        self.battery_file = rospack.get_path('ras_map')+"/battery.txt"
		
        # self.map = cv2.imread(self.map_file, 0)
        self.cell_size = 0.02
        self.b = int(0.5 / self.cell_size)

        maze_file = open(self.maze_file)
        lines = np.array([map(float, line.split(' ')) for line in maze_file])
        max_x = max(max(lines[:, 0]), max(lines[:, 2]))  # max x
        max_y = max(max(lines[:, 1]), max(lines[:, 3]))  # max y
        # min_x = min(min(lines[:,0]), min(lines[:, 2])) # min x
        # min_y = min(min(lines[:,1]), min(lines[:, 3])) # min y
        self.w = int(np.ceil(max_x / self.cell_size))
        self.h = int(np.ceil(max_y / self.cell_size))
        
        lines_img = (lines / self.cell_size).astype(int)
        mp = np.zeros((self.h, self.w))
        for ln in lines_img:
            cv2.line(mp, (ln[0], ln[1]), (ln[2], ln[3]), 120, 1)
        self.map_cells = np.where(mp==120)
        self.map_cells = np.array([self.map_cells[0], self.map_cells[1]])
        # if self.map is None:
        #     self.map = mp
        self.map = mp
        cv2.imwrite(self.map_file, self.map)
        cv2.imwrite(self.map_file_th, cv2.flip(self.map, 0))
        cv2.imwrite(self.path_map+"/map_image/init_map.png", cv2.flip(self.map, 0))

    def update(self):
        if self.scan is None or self.pose2d is None:
            return

        pose_cell = np.array([int(self.pose2d[0]/self.cell_size), int(self.pose2d[1]/self.cell_size)])
        if pose_cell[0] < 0 or pose_cell[1] < 0 or pose_cell[0] > self.w or pose_cell[1] > self.h:
            return

        pxy = np.array([self.scan[0] * np.cos(self.scan[1] + self.pose2d[2]),
                        self.scan[0] * np.sin(self.scan[1] + self.pose2d[2])]) + self.pose2d[:2, np.newaxis]
        p = (pxy / self.cell_size).astype(int)
        im = cv2.imread(self.map_file, 0).astype(int)

        b = self.readBattery()

        err = self.getErr(self.map_cells, p)
        if err < 4:
            p = boundp(p, self.h, self.w)
            im = prepIm(im, p, pose_cell, b).astype(int)
            self.map = im
            cv2.imwrite(self.map_file, self.map)
            th = np.zeros((self.h, self.w)).astype(np.uint8)
            th[im>threshold] = 255
            cv2.imwrite(self.map_file_th, cv2.flip(th, 0))
            try:
    			self.img_pub.publish(self.bridge.cv2_to_imgmsg(th, "8UC1"))
            except CvBridgeError as e:
    			print(e)

        drawMap(im, pose_cell, 'map')

    def getErr(self, map_cells, scan_cells):
        p = scan_cells.T.astype(np.float32)
        m = map_cells.T.astype(np.float32)
        nn = NearestNeighbors(n_neighbors=1).fit(m)
        d, i = nn.kneighbors(p)
        return np.mean(d)

    def readBattery(self):
        try:
            file = open(self.battery_file)
            return (np.array([map(float, line.split(' ')) for line in file])/self.cell_size).astype(int)
        except IOError:
            return None
    
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

    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
		try:
				m.update()
				rate.sleep()
		except rospy.ROSInterruptException:
			cv2.destroyAllWindows()
			pass
		except KeyboardInterrupt:
			cv2.destroyAllWindows()
			#os.remove(m.map_file)
			#os.remove(m.map_file_th)
			break
