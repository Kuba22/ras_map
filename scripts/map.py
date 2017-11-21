import cv2
import numpy as np
import time


class Pose:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Map:
    def __init__(self):
        self.pose = None
        self.scan = None
        self.map_file = "map_image/lab_maze_2017.png"
        self.maze_file = "maze.txt"

        self.map = cv2.imread(self.map_file, 0)
        self.cell_size = 0.01
        if self.map is None:
            maze_file = open(self.maze_file)
            lines = np.array([map(float, line.split(' ')) for line in maze_file])
            lines_img = (lines / self.cell_size).astype(int)
            # min_x = min(min(lines[:,0]), min(lines[:, 2])) # min x
            # min_y = min(min(lines[:,1]), min(lines[:, 3])) # min y
            max_x = max(max(lines[:, 0]), max(lines[:, 2]))  # max x
            max_y = max(max(lines[:, 1]), max(lines[:, 3]))  # max y
            w = int(np.ceil(max_x / self.cell_size))
            h = int(np.ceil(max_y / self.cell_size))
            self.map = np.zeros((w, h))
            for ln in lines_img:
                cv2.line(self.map, (ln[0], ln[1]), (ln[2], ln[3]), 255, 1)
            cv2.imwrite(self.map_file, self.map)
        cv2.imshow('image', self.map)
        cv2.waitKey(1)

    def update(self):
        pxy = np.array([self.scan[0] * np.cos(self.scan[1] + self.pose2d[2]),
                        self.scan[0] * np.sin(self.scan[1] + self.pose2d[2])]) + self.pose2d[:2, np.newaxis]
        p = (pxy / self.cell_size).astype(int)
        print p
        print self.pose2d
        im = cv2.imread(self.map_file, 0).astype(int)
        cv2.line(im,
                 (int(self.pose2d[0]/self.cell_size)+5, int(self.pose2d[1]/self.cell_size)-5),
                 (int(self.pose2d[0]/self.cell_size)-5, int(self.pose2d[1]/self.cell_size)+5), 255, 1)
        cv2.line(im,
                 (int(self.pose2d[0] / self.cell_size) + 5, int(self.pose2d[1] / self.cell_size) + 5),
                 (int(self.pose2d[0] / self.cell_size) - 5, int(self.pose2d[1] / self.cell_size) - 5), 255, 1)
        im[(p[1], p[0])] += 10
        im[im > 255] = 255
        im[im < 0] = 0  # check if this does what i want it to do
        self.map = im.astype(np.uint8)
        cv2.imwrite(self.map_file, self.map)

    def setPose2d(self, pose):
        self.pose2d = np.array([pose.x, pose.y, 2 * np.arcsin(pose.z)])

    def scanCallback(self, msg):
        self.scan = msg
        print "Received scan"

    def poseCallback(self, msg):
        self.pose = msg.pose
        print "Received pose"

A = np.array([[0,1,2],[3,4,5],[6,7,8]])
B = np.array([[1, 1, 2],[2, 2, 2]])
A[(B[0], B[1])]+=1
A[A > 8] = 8

pi = 3.14159

if __name__ == '__main__':
    m = Map()

    try:
        while True:
            m.pose = Pose(0.2, 1.4, 0.0)
            m.setPose2d(m.pose)
            m.scan = np.array([[0.3, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.37],
                               [pi / 180 * 0, pi / 180 * 1, pi / 180 * 2, pi / 180 * 3, pi / 180 * 4, pi / 180 * 5,
                                pi / 180 * 6, pi / 180 * 7]])

            m.update()

            cv2.imshow('image', cv2.resize(m.map, None, fx=2, fy=2))
            cv2.waitKey(10)
            time.sleep(0.4)
    finally:
        cv2.destroyAllWindows()
