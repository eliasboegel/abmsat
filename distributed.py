"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer, random
from agent import AgentDistributed
from single_agent_planner import get_sum_of_cost

class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = []
        # T.B.D.
        
    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations.
        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants       
        start_time = timer.time()
        result = []
        self.CPU_time = timer.time() - start_time
        
        
        # Create agent objects with AgentDistributed class
        agents = []
        for i in range(self.num_of_agents):
            newAgent = AgentDistributed(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i)
            agents.append(newAgent)

        all_finished = False
        for agent in agents: all_finished = all_finished or agent.is_finished()

        t = 0
        while not all_finished:
            # Iterate over all unique combinations of collisions
            for i in range(len(agents) - 1):
                for j in range(i + 1, len(agents)):
                    a1_pos1 = agents[i].position_at(t)
                    a2_pos1 = agents[j].position_at(t)
                    
                    a1_pos2 = agents[i].position_at(t+1)
                    a2_pos2 = agents[j].position_at(t+1)
                    vertex_collided = 1 if a1_pos2 == a2_pos2 else 0
                    edge_collided = 1 if a1_pos1 == a2_pos2 and a2_pos1 == a1_pos2 else 0

                    # Handle collision
                    if vertex_collided or edge_collided:

                        # Momentum checks
                        if agents[i].momentum > agents[j].momentum:
                            loc1 = agents[j].position_at(t)
                            loc2 = a2_pos2
                            new_constraint = {'agent': j,
                                              'loc': [loc1, loc2],
                                              'timestep': t}
                            agents[j].plan_path(new_constraint)
                        elif agents[i].momentum < agents[j].momentum:
                            loc1 = agents[i].position_at(t)
                            loc2 = a2_pos2
                            new_constraint = {'agent': i,
                                              'loc': [loc1, loc2],
                                              'timestep': t}
                            agents[i].plan_path(new_constraint)
                        else:
                            agent_selector = random.randrange(0, 2) # Random int, either 0 or 1
                            a1_id = i if agent_selector else j
                            a2_id = j if agent_selector else i

                            loc1 = agents[a1_id].position_at(t)
                            loc2 = a2_pos2
                            new_constraint = {'agent': i,
                                              'loc': [loc1, loc2],
                                              'timestep': t}
                            agents[i].plan_path(new_constraint)
            
            for agent in agents:
                agent.move_with_plan()
                

            t = t + 1
            Animation(my_map, starts, goals, augmented_paths)
            plt.cla()
            plt.pause() # could put this inside the visualize class

        plt.show()
        
        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)
        
        return result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)