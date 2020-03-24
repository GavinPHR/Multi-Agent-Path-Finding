#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
from typing import List, Tuple
import numpy as np
from scipy.optimize import linear_sum_assignment

from .agent import Agent

# Hungarian algorithm for global minimal cost
def min_cost(starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
    # The number of start positions must be equal to the number of goal positions
    assert(len(starts) == len(goals))

    sqdist = lambda x, y: (x[0]-y[0])**2 + (x[1]-y[1])**2
    cost_vec = []
    for start in starts:
        for goal in goals:
            cost_vec.append(sqdist(start, goal))
    n = len(starts)
    cost_mtx = np.array(cost_vec).reshape((n, n))
    row_ind, col_ind = linear_sum_assignment(cost_mtx)
    agents = []
    for i, start in enumerate(starts):
        agents.append(Agent(start, goals[col_ind[i]]))
    return agents


# Greedily choosing closest distance
def greedy_assign(starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
    # The number of start positions must be equal to the number of goal positions
    assert(len(starts) == len(goals))

    goal_set = set(goal for goal in goals)
    sqdist = lambda x, y: (x[0]-y[0])**2 + (x[1]-y[1])**2
    agents = []
    for start in starts:
        closest = float('inf')
        closest_goal = None
        for goal in goal_set:
            d = sqdist(start, goal)
            if d < closest:
                closest = d
                closest_goal = goal
        goal_set.remove(closest_goal)
        agents.append(Agent(start, closest_goal))
    return agents



