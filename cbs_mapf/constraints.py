#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
from typing import Dict, List, Tuple
from copy import deepcopy

from .agent import Agent

'''
Emulated dictionary of dictionaries
'''
class Constraints:

    def __init__(self):
        #                                   time,         obstacles
        self.agent_constraints: Dict[Agent: Dict[int, List[Tuple[int, int]]]] = dict()

    '''
    Deepcopy self with additional constraints
    '''
    def fork(self, agent: Agent, dynamic_obstacles: Dict[int, List[Tuple[int, int]]]) -> 'Constraints':
        constraints_copy = deepcopy(self.agent_constraints)
        agent_obstacles = constraints_copy.setdefault(agent, dict())
        for time, obstacles in dynamic_obstacles:
            agent_obstacles.setdefault(time, []).extend(obstacles)
        new_constraints = Constraints()
        new_constraints.agent_constraints = constraints_copy
        return new_constraints

    def setdefault(self, key, default):
        return self.agent_constraints.setdefault(key, default)

    def __getitem__(self, agent):
        return self.agent_constraints[agent]

    def __iter__(self):
        for key in self.agent_constraints:
            yield key


