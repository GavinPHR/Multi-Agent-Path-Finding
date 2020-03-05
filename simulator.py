import os
import cv2
import time
import pickle
import numpy as np
from multi_agent_planner import MultiAStar
from typing import Dict, Tuple
import copy
from config import ROBOT_RADIUS

'''
See README for more info.
'''
class Simulator:

    def __init__(self, param: str, start: Dict[int, Tuple[int]], goal: Dict[int, Tuple[int]]):
        self.multi_planner = MultiAStar()
        self.multi_planner.naive_plan(start, goal)
        self.padded_path = copy.deepcopy(self.multi_planner.planning(start, goal))
        self.pad_path()

        dirname = os.path.abspath(__file__ + "/..")
        self.frame_orig = cv2.imread(os.path.join(dirname, 'saved', param +'.png'))
        self.obsts = pickle.load(open(os.path.join(dirname, 'saved', param +'.p'), "rb"))
        self.frame_obst = self.frame_orig.copy()
        self.draw_rect(np.array([np.array(v) for v in self.obsts.values()]))
        self.draw_grid(self.frame_obst)



    def pad_path(self):
        max_length = max(len(path) for path in self.padded_path.values())
        for id, path in self.padded_path.items():
            if len(path) < max_length:
                path += [path[-1]]*(max_length-len(path))


    def draw_rect(self, pts_arr: np.ndarray) -> None:
        for pts in pts_arr:
            cv2.rectangle(self.frame_obst, tuple(pts[0]), tuple(pts[1]), (0, 0, 255), thickness=3)

    def draw_grid(self, frame):
        for x, y, b in self.multi_planner.single_planner.get_grid():
            if b:
                cv2.circle(frame, (int(x), int(y)), 1, (0, 0, 255))
            else:
                cv2.circle(frame, (int(x), int(y)), 1, (0, 255, 0))

    def draw_path(self, frame, xys):
        # cv2.polylines(frame, np.int32([list(zip(self.rx, self.ry))]), False, (0, 255, 0), thickness=3)
        for x, y in xys:
            cv2.circle(frame, (int(x), int(y)), 4, (255, 0, 0))

    '''
    Press 'q' to exit
    '''
    def start(self):
        cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame', (1280, 720))
        try:
            i = 0
            while True:
                frame = copy.deepcopy(self.frame_obst)
                for id in self.padded_path:
                    cv2.circle(frame, self.padded_path[id][i], ROBOT_RADIUS, ((id==1)*255, (id==2)*255, (id==3)*255))
                cv2.imshow('frame', frame)
                k = cv2.waitKey(100) & 0xFF  # '& 0xFF' for 64-bit compatibility
                if k == ord('q'):
                    break
                i += 1
        except Exception:
            cv2.waitKey(0)
            cv2.destroyAllWindows()

    def show(self):
        frame = copy.deepcopy(self.frame_obst)
        for xys in self.multi_planner.naive_path.values():
            self.draw_path(frame, xys)
        cv2.imshow('frame', frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
'''
Test code.
'''
if __name__ == '__main__':
    testStart = {
        1: (331, 165),
        2: (1530, 565),
        # 3: (315, 951)
    }

    testGoal = {
        1: (955, 819),
        2: (337, 565),
        # 3: (1530, 565)
    }
    t1 = time.time()
    r = Simulator('room0', testStart, testGoal)  # Load saved room and obstacles
    t2 = time.time()
    print(t2-t1)
    r.start()
    # r.show()
    t3 = time.time()
    print(t3-t2)

