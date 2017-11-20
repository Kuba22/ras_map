#!/usr/bin/env python
import rospy
import rospkg
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose
from cv_bridge import CvBridge

class Map:
	def __init__(self):
		rospy.init_node("map")
		self.pose = None
		self.scan = None
		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scanCallback)
		self.pose_sub = rospy.Subscriber("/localization/pose", PoseStamped, self.poseCallback)

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

	def update(self):
		# get cell coords from scan
		pxy = np.array([self.scan[0]*np.cos(self.scan[1]+self.pose2d[2]), self.scan[0]*np.sin(self.scan[1]+self.pose2d[2])])+self.pose2d[:2, np.newaxis]
		print pxy

	def setPose2d(self, pose):
		self.pose2d = np.array([pose.pose.position.x, pose.pose.position.y, 2*np.arcsin(pose.pose.orientation.z)])

	def scanCallback(self, msg):
		self.scan = msg
		print "Received scan"

	def poseCallback(self, msg):
		self.pose = msg.pose
		print "Received pose"

pi = 3.14159

if __name__=='__main__':
	try:
		m = Map()
		rate = rospy.Rate(5)
		
		while not rospy.is_shutdown():
			m.pose = PoseStamped()
			m.pose.pose = Pose()
			m.pose.pose.position.x = 0.2
			m.pose.pose.position.y = 1.4
			m.pose.pose.position.z = 0.0
			m.pose.pose.orientation.z = 0.0
			m.pose.pose.orientation.w = 1.0
			m.setPose2d(m.pose)
			m.scan = np.array([[0.3, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.37],[pi/180*0, pi/180*1, pi/180*2, pi/180*3, pi/180*4, pi/180*5, pi/180*6, pi/180*7]])

			m.update()

			cv2.imshow('image', cv2.resize(m.map, None, fx=2, fy=2))
			rate.sleep()
	except rospy.ROSInterruptException:
		cv2.destroyAllWindows()
		pass
