import time as timer
from single_agent_planner import compute_heuristics, compute_heuristics_goals, a_star, get_sum_of_cost


class IndependentSolver(object):
    """A planner that plans for each robot independently."""

    def __init__(self, my_map, starts, goals, heuristics_func=None):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics
        self.heuristics = []
        if heuristics_func == 'old':
            for goal in self.goals:
                self.heuristics.append(compute_heuristics(my_map, goal))
        elif heuristics_func == 'goals':
            for goal in self.goals:
                self.heuristics.append(compute_heuristics_goals(my_map, goal, goals))
        else:
            for goal in self.goals:
                self.heuristics.append(compute_heuristics(my_map, goal))


    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []

        ##############################
        # Task 0: Understand the following code (see the lab description for some hints)

        for i in range(self.num_of_agents):  # Find path for each agent
            print(f"----------------\nSolving for agent: {i}")
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, [])
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

        ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))

        return result
