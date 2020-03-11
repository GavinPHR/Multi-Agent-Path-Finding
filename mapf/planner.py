#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
from typing import List, Tuple
from heapq import heappush, heappop
from .assigner import greedy_assign
# The low level planner for CBS is the Space-Time A* planner
# https://github.com/GavinPHR/Space-Time-AStar
from stastar.planner import Planner as STPlanner
from .constraint_tree import CTNode

class Planner:

    def __init__(self, grid_size: int,
                       robot_radius: int,
                       static_obstacles: List[Tuple[int, int]]):

        self.robot_radius = robot_radius
        self.st_planner = STPlanner(grid_size, robot_radius, static_obstacles)

    '''
    You can use your own assignment function, the default algorithm greedily assigns
    the closest goal to each start.
    '''
    def plan(self, starts: List[Tuple[int, int]], goals: List[Tuple[int, int]], assign=greedy_assign):
        # Do goal assignment
        agents = assign(starts, goals)

        constraints = dict()  # Should be an empty dict here
        solution = dict()

        # Compute path for each agent using low level planner
        for agent in agents:
            solution[agent] = self.st_planner.plan(agent.start,
                                                   agent.goal,
                                                   constraints.setdefault(agent, dict()))

        node = CTNode(constraints, solution)

        # Min heap for quick extraction
        open = [node]

        while open:
            best = heappop(open)

    def validate_paths(self):
        raise NotImplementedError
    def safe_something(self):
        raise NotImplementedError

