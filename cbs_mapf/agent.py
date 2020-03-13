#!/usr/bin/env python3
'''
Author: Haoran Peng
Email: gavinsweden@gmail.com
'''
from typing import Tuple
import numpy as np


class Agent:

    def __init__(self, start: Tuple[int, int], goal: Tuple[int, int]):
        self.start = np.array(start)
        self.goal = np.array(goal)

    # Uniquely identify an agent with its start position
    def __hash__(self):
        return int(str(self.start[0]) + str(self.start[1]))

    def __eq__(self, other: 'Agent'):
        return np.array_equal(self.start, other.start) and \
               np.array_equal(self.goal, other.goal)

    def __str__(self):
        return str(self.start.tolist())

    def __repr__(self):
        return self.__str__()
