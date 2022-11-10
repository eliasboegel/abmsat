import time as timer
from single_agent_planner import compute_heuristics, compute_heuristics_goals, a_star, get_sum_of_cost, get_location, constrain_path


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

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

        # compute heuristics for the low-level search
        if heuristics_func == None:
            self.heuristics = []
            for goal in self.goals:
                self.heuristics.append(compute_heuristics(my_map, goal))
        else:
            self.heuristics = heuristics_func

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        # Find path for each agent
        for i in range(self.num_of_agents):  
            
            # Running a* for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)
            constraint = [] 
            
            # print(f'agent {i} has path: {path}')
            for k in range(i+1, self.num_of_agents):
                constraint = constraint + (constrain_path(path, k))

            constraints = constraints + constraint

            ##############################
        # print(f'this is result {result}')
        self.CPU_time = timer.time() - start_time
        
        # print("\nFound a solution!\n")
        # print("CPU time (s):    {:.10f}".format(self.CPU_time))
        # print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        # print(result)
        return result
