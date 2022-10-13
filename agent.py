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


    def try_path(self, map, constraint, t, time_dependent=True):
        # print(f"pos_at_replan {self.position_at(t)}")
        print('try path triggered')
        print(f'current planned path for agent {self.id}: {self.planned_path}')
        self.last_tried_path = a_star(map, self.pos, self.goal, self.heuristics, self.id, constraint, time_dependent=time_dependent)
        return self.last_tried_path

    def use_path(self, path, t):
        print('use path triggered')
        print(f'current planned path for agent {self.id}: {self.planned_path}')
        self.planned_path = path
        self.planned_path_t = t

    def plan_path(self, constraint, t):
        # For position, you have to use self.pos and not position at. Because position at takes the planned path of
        position = self.position_at(t-1)
        print(f't-1 is {t-1}, planned time is {self.planned_path_t}')
        print(f'current planned path for agent {self.id}: {self.planned_path}')
        self.planned_path = a_star(self.my_map, position, self.goal, self.heuristics, self.id, constraint)
        self.planned_path_t = t
    
    def get_remaining_planned_path(self, t):
        return self.planned_path[t - self.planned_path_t:]


    def move_with_plan(self, t):
        new_pos = self.position_at(t+1)
        curr_pos = self.position_at(t)
        print(f't is {t}\ncurr_pos{curr_pos}\nnew_pos {new_pos}\n')
        curr_move = (new_pos[0] - curr_pos[0], new_pos[1] - curr_pos[1])
        # print(f'curr_pos {curr_pos}')
        # print(f'new_pos {new_pos}')
        # print(f'plan {self.planned_path}\n\n')
        self.pos = new_pos # Retrieve next position along path and move to it
        
        if curr_move == self.last_move and curr_move != (0,0):
            self.momentum += 1
        else:
            self.momentum = 0

        self.last_move = curr_move
        self.path.append(self.pos)

        
    def position_at(self, t):
        if t < 0:
            return self.start
        elif t >= self.planned_path_t:
            if t >= self.planned_path_t + len(self.planned_path):
                return self.goal
            else:
                return self.planned_path[t - self.planned_path_t]
        else:
            return self.path[t]

    def get_last_two_moves(self, t):
        return [self.position_at(t-1), self.position_at(t)]

    def is_on_goal(self):
        return (self.pos == self.goal)