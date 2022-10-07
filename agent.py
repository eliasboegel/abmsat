"""
This file contains the AgentDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

from tracemalloc import start
from single_agent_planner import compute_heuristics, a_star

class AgentDistributed(object):
    """Agent object to be used in the distributed planner."""

    def __init__(self, my_map, start, goal, agent_id):
        """
        my_map   - list of lists specifying obstacle positions
        starts      - (x1, y1) start location
        goals       - (x1, y1) goal location
        heuristics  - heuristic to goal location
        """

        self.my_map = my_map
        self.start = start
        self.goal = goal
        self.id = agent_id
        self.heuristics = compute_heuristics(self.my_map, self.goal)

        self.pos = self.start
        self.path = [self.start]
        self.distance_traveled = 0
        self.last_move = (0, 0)

        self.planned_path = []
        self.planned_path_t = 0

        self.plan_path(0)

        self.momentum = 0


    def plan_path(self, constraint):
        self.planned_path = a_star(self.my_map, self.pos, self.goal, self.heuristics, constraint, [])
        
    
    def move_with_plan(self, t):
        curr_move = self.planned_path[t - self.planned_path_t + 1] - self.planned_path[t - self.planned_path_t]
        self.pos = self.planned_path[t - self.planned_path_t + 1] # Retrieve next position along path and move to it
        
        if curr_move == self.last_move and curr_move != (0,0):
            self.momentum += 1
        else:
            self.momentum = 0

        self.last_move = curr_move

        
    def position_at(self, t):
        if t < 0:
            return self.start
        elif t > self.planned_path_t:
            if t > self.planned_path_t + len(self.planned_path):
                return self.goal
            else:
                return self.planned_path[t - self.planned_path_t]
        else:
            return self.path[t]


    def is_finished(self):
        return (self.pos == self.goal)