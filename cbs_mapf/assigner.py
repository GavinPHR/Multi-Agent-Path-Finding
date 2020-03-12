#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
from typing import List, Tuple

from .agent import Agent

# Currently greedily choosing closest distance
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



