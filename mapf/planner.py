#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
from typing import List, Tuple

from .assigner import greedy_assign
# The low level planner for CBS is the Space-Time A* planner
# https://github.com/GavinPHR/Space-Time-AStar
from stastar.planner import Planner as STPlanner

class Planner:

    def __init__(self, grid_size: int,
                       robot_radius: int,
                       static_obstacles: List[Tuple[int, int]]):

        self.robot_radius = robot_radius
        self.st_planner = STPlanner(grid_size, robot_radius, static_obstacles)

    def plan(self, starts: List[Tuple[int, int]], goals: List[Tuple[int, int]], assign=greedy_assign):
        # Do goal assignment
        goals = assign(starts, goals)
