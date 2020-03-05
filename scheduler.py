from typing import Dict, List, Tuple
import copy
import math
from config import ROBOT_RADIUS, NUM2RESERVE, MAX_ITER
from collections import defaultdict

class Scheduler:

    def __init__(self):
        # Number of checkpoints to reserve
        self.reservation_table = defaultdict(set)

    def schedule(self, naive_path: Dict[int, List[Tuple[int]]]):
        naive_path = copy.deepcopy(naive_path)
        scheduled_path = dict((id, []) for id in naive_path)
        i = 0
        while i < MAX_ITER and naive_path:
            # print(naive_path)
            for id in list(naive_path.keys()):
                if not naive_path[id]:
                    del naive_path[id]
            self.reservation_table = defaultdict(set)
            for id, path in naive_path.items():
                self.reservation_table[id] = {path[0]}
            for id, path in naive_path.items():
                if self.reserve(path[:NUM2RESERVE], id):
                    scheduled_path[id].extend(path[:NUM2RESERVE])
                    naive_path[id] = path[NUM2RESERVE:]
                elif scheduled_path[id]:
                    scheduled_path[id].extend([scheduled_path[id][-1]]*NUM2RESERVE)
                else:
                    continue
            i += 1
        if i == MAX_ITER:
            raise RuntimeError('Multi-Agent Scheduling Failed')
        return scheduled_path

    def reserve(self, checkpoints, id):
        flag = True
        for checkpoint in checkpoints:
            if not self.check_reservation(checkpoint, id):
                flag = False
                break
        if not flag:
            return False
        else:
            self.reservation_table[id].update(checkpoints)
            return True

    def check_reservation(self, checkpoint, id):
        dist = lambda c1, c2: math.sqrt((c1[0]-c2[0])**2+(c1[1]-c2[1])**2)
        flag = True
        for i, table_entry in self.reservation_table.items():
            if i == id:
                continue
            for reserved_checkpoint in table_entry:
                if dist(checkpoint, reserved_checkpoint) < 2*ROBOT_RADIUS + 5:
                    flag = False
                    break
            if not flag:
                break
        return flag





path = {1: [(330, 161), (350, 181), (370, 201), (390, 221), (410, 241), (430, 261), (450, 281), (450, 301), (450, 321), (450, 341), (450, 361), (450, 381), (450, 401), (470, 421), (470, 441), (490, 461), (510, 481), (530, 501), (550, 521), (570, 541), (590, 561), (610, 581), (630, 581), (650, 581), (670, 581), (690, 581), (710, 581), (730, 581), (750, 601), (770, 601), (790, 601), (810, 621), (830, 641), (850, 661), (870, 681), (890, 701), (910, 721), (910, 741), (930, 761), (930, 781), (930, 801), (950, 821)],
        2: [(1530, 561), (1510, 561), (1490, 561), (1470, 561), (1450, 561), (1430, 561), (1410, 561), (1390, 561), (1370, 561), (1350, 561), (1330, 561), (1310, 561), (1290, 561), (1270, 561), (1250, 561), (1230, 561), (1210, 561), (1190, 561), (1170, 561), (1150, 561), (1130, 561), (1110, 561), (1090, 561), (1070, 561), (1050, 561), (1030, 561), (1010, 561), (990, 561), (970, 561), (950, 561), (930, 561), (910, 561), (890, 561), (870, 561), (850, 561), (830, 561), (810, 561), (790, 561), (770, 561), (750, 561), (730, 561), (710, 561), (690, 561), (670, 561), (650, 561), (630, 561), (610, 561), (590, 561), (570, 561), (550, 561), (530, 561), (510, 561), (490, 561), (470, 561), (450, 561), (430, 561), (410, 561), (390, 561), (370, 561), (350, 561), (330, 561)],
        # 3: [(310, 961), (330, 961), (350, 961), (370, 961), (390, 961), (410, 961), (430, 961), (450, 961), (470, 961), (490, 961), (510, 961), (530, 961), (550, 961), (570, 961), (590, 961), (610, 961), (630, 961), (650, 961), (670, 961), (690, 941), (710, 941), (730, 941), (750, 941), (770, 941), (790, 941), (810, 921), (830, 901), (850, 881), (870, 861), (890, 841), (910, 821), (930, 801), (950, 781), (970, 761), (990, 741), (1010, 721), (1030, 701), (1030, 681), (1050, 661), (1070, 641), (1090, 621), (1110, 601), (1130, 601), (1150, 601), (1170, 601), (1190, 601), (1210, 601), (1230, 601), (1250, 601), (1270, 601), (1290, 601), (1310, 601), (1330, 601), (1350, 601), (1370, 601), (1390, 601), (1410, 601), (1430, 601), (1450, 601), (1470, 581), (1490, 581), (1510, 581), (1530, 561)]
        }

if __name__ == '__main__':
    s = Scheduler()
    print(s.schedule(path))
