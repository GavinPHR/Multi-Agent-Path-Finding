from single_agent_planner import AStarPlanner
import scheduler
from config import GRID_SIZE, ROBOT_RADIUS
from typing import List, Tuple, Dict

class MultiAStar:

	def __init__(self):
		# Initialize a single agent planner. Obstacles should already be serialized.
		self.single_planner = AStarPlanner([], [], GRID_SIZE, ROBOT_RADIUS, True)
		self.naive_path = dict()

	def naive_plan(self, start: Dict[int, Tuple[int]], goal: Dict[int, Tuple[int]]):
		try:
			assert(set(start.keys()) == set(goal.keys()))
		except ValueError:
			print('Some table does not have a start/goal position.')
			raise

		for id in start.keys():
			sx, sy, gx, gy = start[id][0], start[id][1], goal[id][0], goal[id][1]
			self.naive_path[id] = self.single_planner.planning(sx, sy, gx, gy)


testStart = {
	1: (331, 165),
	2: (1530, 565),
	3: (315, 951)
}

testGoal = {
	1: (955, 819),
	2: (337, 565),
	3: (1530, 565)
}

p = MultiAStar()
p.naive_plan(testStart, testGoal)
print(p.naive_path)

