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

        self.last_tried_path = []
        self.planned_path = []
        self.planned_path_t = 0

        self.plan_path([], 0)

        self.momentum = 0
        self.moves_back = ("", None)

        self.curr_constraints = []
        self.obstructed_map = my_map



    def try_path(self, map, constraint, t, time_dependent=True, goal=None):
        goal = self.goal if goal is None else goal
        self.last_tried_path = a_star(map, self.pos, goal, self.heuristics, self.id, constraint, time_dependent=time_dependent, init_time=t)

        if self.last_tried_path: # need to update planned_path_t if agent tries another path and this another path works 
            self.planned_path_t = t
        else:
            print('none')

        return self.last_tried_path

    def use_path(self, path, t):
        self.planned_path = path
        self.planned_path_t = t

    def plan_path(self, constraint, t, goal=None, used_map=None):
        # For position, you have to use self.pos and not position at. Because position at takes the planned path of
        goal = self.goal if goal is None else goal
        used_map = self.my_map if used_map is None else used_map
        position = self.pos
        self.planned_path_t = t
        self.planned_path = a_star(used_map, position, self.goal, self.heuristics, self.id, constraint, init_time=t)
        return self.planned_path
    
    def get_remaining_planned_path(self, t):
        return self.planned_path[t - self.planned_path_t:]


    def move_with_plan(self, t):
        new_pos = self.position_at(t+1)
        curr_pos = self.position_at(t)
        curr_move = (new_pos[0] - curr_pos[0], new_pos[1] - curr_pos[1])
        self.pos = new_pos # Retrieve next position along path and move to it
        
        if curr_move == self.last_move and curr_move != (0,0):
            
            self.momentum += 1
        else:
            self.momentum = 0

        self.last_move = curr_move
        self.path.append(self.pos)
        self.obstructed_map = self.my_map
        self.curr_constraints = []

    def position_at(self, t):
        if t < 0:
            return self.start
        elif t > self.planned_path_t and self.planned_path is not None:
            if t >= (self.planned_path_t + len(self.planned_path) if self.planned_path is not None else 0):
                # print(f'returning goal, agent {self.id} has plannt t of {self.planned_path_t}')
                return self.planned_path[-1]
            else:
                # print(f'returning not goal, agent {self.id} has planned t of {self.planned_path_t}, and is t at {t}')
                return self.planned_path[t - self.planned_path_t ]
        else:
            return self.path[t]
    
    def position_at2(self, t):
        t_end = len(self.path) + len(self.planned_path) - 1
        if 0 <= t:
            return self.start
        elif 0 < t <= self.planned_path_t:
            return self.path[t]
        elif self.planned_path_t < t <= t_end:
            return self.planned_path[t - self.planned_path_t + 1]
        elif t_end < t:
            return self.planned_path[-1]

    def get_last_two_moves(self, t):
        return [self.position_at(t-1), self.position_at(t)]

    def is_on_goal(self):
        return (self.pos == self.goal)