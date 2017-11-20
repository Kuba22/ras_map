#!/usr/bin/env python
import rospy
import rospkg
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class Map:
	def __init__(self):
		rospy.init_node("map")
		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scanCallback)
		rospack = rospkg.RosPack()
		path_maze = rospack.get_path('ras_maze_map')
		path_map = rospack.get_path('ras_map')
		self.map = cv2.imread(path_map + "/map_image/lab_maze_2017.png", 0)
		self.cell_size = 0.01
		if self.map is None:
			maze_file = open(path_maze + '/maps/lab_maze_2017.txt')
			lines = np.array([map(float, line.split(' ')) for line in maze_file])
			lines_img = (lines/self.cell_size).astype(int)
			#min_x = min(min(lines[:,0]), min(lines[:, 2])) # min x
			#min_y = min(min(lines[:,1]), min(lines[:, 3])) # min y
			max_x = max(max(lines[:,0]), max(lines[:, 2])) # max x
			max_y = max(max(lines[:,1]), max(lines[:, 3])) # max y
			w = int(np.ceil(max_x/self.cell_size))
			h = int(np.ceil(max_y/self.cell_size))
			self.map = np.zeros((w, h), dtype="uint8")
			for ln in lines_img:				
				cv2.line(self.map, (ln[0], ln[1]), (ln[2], ln[3]), 255, 1)
			cv2.imwrite(path_map + "/map_image/lab_maze_2017.png", self.map)
		#cv2.imshow('image', self.map)
		#cv2.waitKey(0)
		#cv2.destroyAllWindows()

	def scanCallback(self, msg):
		self.scan = msg

	def poseCallback(self, msg):
		self.pose = msg.pose
		

if __name__=='__main__':
	try:
		node = Map()
		rate = rospy.Rate(10)
		#while not rospy.is_shutdown():
			# update map
			#rate.sleep()
	except rospy.ROSInterruptException:
		pass
