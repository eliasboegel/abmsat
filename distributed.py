"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer, random, copy
from agent import AgentDistributed
from visualize import Animation
import matplotlib.pyplot as plt
from single_agent_planner import get_sum_of_cost, constrain_whole_path

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
        
    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations.
        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants       
        fix, ax = plt.subplots()
        start_time = timer.time()
        result = []
        all_finished = False
        self.CPU_time = timer.time() - start_time
        
        
        # Create list of agent objects with AgentDistributed class
        print('\n\ninitializing all agents....')
        agents = []
        for i in range(self.num_of_agents):
            newAgent = AgentDistributed(self.my_map, self.starts[i], self.goals[i], i)
            agents.append(newAgent)


        # Simulation loop
        t = 0
        while not all_finished:
            print('\n\n\n\n')
            # Iterate over all unique combinations of agents
            for i in range(len(agents) - 1):
                for j in range(i + 1, len(agents)):
                    print(f'planned path of agent{i}: {agents[i].planned_path}\nand it has recorded path of: {agents[i].path}\n')
                    print(f'planned path of agent{j}: {agents[j].planned_path}\nand it has recorded path of: {agents[j].path}')

                    # Retreieving locations at t and t+1 for both agents in the combination
                    a1_pos1 = agents[i].position_at(t)
                    a2_pos1 = agents[j].position_at(t)
                    a1_pos2 = agents[i].position_at(t+1)
                    a2_pos2 = agents[j].position_at(t+1)
                    
                    # Generating collision booleans
                    vertex_collided = 1 if a1_pos2 == a2_pos2 else 0
                    edge_collided = 1 if a1_pos1 == a2_pos2 and a2_pos1 == a1_pos2 else 0

                    # Handle collision
                    if vertex_collided or edge_collided:
                        print(f'agents {i} & {j} have a collision!\nVertex? {vertex_collided}\nedge? {edge_collided}')
                        # TODO: ADDITIONAL RULES FOR HANDLING GOAL

                        
                        # if agents[i].is_on_goal() or agents[j].is_on_goal():
                        #     a1_id = i if agents[j].is_on_goal() else j # Agent 1 needs to pass agent 2
                        #     a2_id = j if agents[j].is_on_goal() else i # Agent 2 is on goal

                        #     a2_pos = agents[a2_id].position_at(t)

                        #     evade_a2_map = copy.deepcopy(self.my_map)
                        #     evade_a2_map[a2_pos[0]][a2_pos[1]] = True
                        #     #evade_a2_constraint = {'agent': a1_id,
                        #     #                       'loc': [a2_pos, a2_pos],
                        #     #                       'timestep': t}
                        #     a1_path_evade_a2 = agents[a1_id].try_path(evade_a2_map, [], t, time_dependent=False)
                            
                        #     if a1_path_evade_a2: # If a1 was able to replan around a2
                        #         agents[a1_id].use_path(a1_path_evade_a2, t)
                        #         print(f"a1 replans around a2: {a1_path_evade_a2}")
                        #     else: # If evasion of a2 isnt possible
                        #         evade_a1path_constraints = constrain_whole_path(agents[a1_id].get_remaining_planned_path(t), a2_id)
                        #         a2_evade_a1path = agents[a2_id].try_path(self.my_map, evade_a1path_constraints, t)
                        #         print(f"a2 replans around a1 path: {a2_evade_a1path}")
                        #         if a2_evade_a1path: # Possible for a2 to move out of a1 path
                        #             agents[a2_id].use_path(a2_evade_a1path, t)
                        #         else: # Dead end, both need to move back
                        #             agents[a2_id].use_path([agents[a1_id].position_at(t)], t)
                        #             agents[a1_id].use_path([agents[a1_id].position_at(t-1)], t)
                        # Momentum handling
                        if agents[i].momentum != agents[j].momentum: 
                            a1_id = i if agents[i].momentum > agents[j].momentum else j
                            a2_id = j if agents[i].momentum > agents[j].momentum else i

                            # Loser agent is given constraints and has to replan path
                            loc1 = agents[a1_id].position_at(t)
                            loc2 = agents[a1_id].position_at(t+1)
                            new_constraint = {'agent': a2_id,
                                              'loc': [loc1, loc2],
                                              'timestep': t}
                            agents[a2_id].plan_path([new_constraint], t)
                            print(f'aa agent{a2_id} planned a new path: {agents[a2_id].planned_path}')
                        else:
                            # Random loser if both agents have same momentum
                            agent_selector = 0#random.randrange(0, 2) # Random int, either 0 or 1
                            a1_id = i if agent_selector else j
                            a2_id = j if agent_selector else i
                            
                            loc1 = agents[a1_id].position_at(t)
                            loc2 = agents[a1_id].position_at(t+1)
                            new_constraint = {'agent': a2_id,
                                              'loc': [loc1, loc2],
                                              'timestep': t}
                            agents[a2_id].plan_path([new_constraint], t)
                            print(f'cc agent{a2_id} planned a new path: {agents[a2_id].planned_path}')
                            
            n_finished = 0 
            print('\n\nmoving all agents with their current plan.....')
            for agent in agents:
                print(f'agent{agent.id} has moved from \n{agent.pos} to')
                agent.move_with_plan(t)
                n_finished = n_finished + agent.is_on_goal()
                print(f'{agent.pos}\nnow it has a recorded path of {agent.path}\n')
            # Ending simulation if all agents are on goals
            if n_finished == len(agents): all_finished = True
            
            t = t + 1
            animation_paths = []
            for i in range(len(agents)):
                animation_paths.append(agents[i].get_last_two_moves(t))
            # print(f'animate paths {animation_paths}')
            
            #a = Animation(self.my_map, self.starts, self.goals, animation_paths)
            #a.animate_once(animation_paths)
            
        for agent in agents:
            result.append(agent.path)
        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.6f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)
        
        return result