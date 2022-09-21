import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, get_location


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
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
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            print(f'\n-------------Planning for agent {i}-------------')
            
            # print(f'constraints applying to agent {i}: {constraints}\n\n')
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            constraint = [] 
            print(f'agent {i} has path: {path}')
            for j in range(len(path)):
                for k in range(i+1, self.num_of_agents):
                    if j == (len(path)-1):
                        for l in range(1,3):
                            loc = get_location(path, j+l)
                            agent = k
                            constraint_dict = {'agent': agent,
                                               'loc': [loc],
                                               'timestep': j+l}     
                            constraint.append(constraint_dict)
                        continue
                    loc1 = get_location(path, j)
                    loc2 = get_location(path, j+1)
                    agent = k
                    constraint_dict = {'agent': agent,
                                       'loc': [loc1,loc2],
                                       'timestep': j+1}   
                    constraint.append(constraint_dict)
            print(f'constraints from agent{i}: {constraint}')
            constraints = constraints + constraint
            # print(f'constraints: {constraints}')

            ##############################
        print(f'this is result {result}')
        self.CPU_time = timer.time() - start_time

        print("\nFound a solution!\n")
        print("CPU time (s):    {:.10f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
