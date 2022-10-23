"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer, random, copy
from agent import AgentDistributed
from visualize import Animation
import matplotlib.pyplot as plt
from single_agent_planner import get_sum_of_cost, constrain_path

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

    def update_others(self, loser_newplan, init_constraints, ignored_ids):
        constraints = init_constraints

        # Creating the other_agents list
        other_agents = list(filter(lambda id: id not in ignored_ids, [*range(len(self.agents))]))

        # Sort 
        sorted_other_agent_ids = sorted(other_agents, key=lambda agent_id: self.agents[agent_id].momentum, reverse=True)
        # Update other agents in descending momentum priority order
        print(f'                                      somethignnirwig {sorted_other_agent_ids}')
        for other_agent_id in sorted_other_agent_ids:
            constraints = constraints + constrain_path([loser_newplan[0]], other_agent_id, self.t)
            new_path = self.agents[other_agent_id].plan_path(constraints, self.t)
            new_ignores = [other_agent_id] #+ ignored_ids
            print(f'    new ignoressss {new_ignores}')
            self.update_others(new_path, constraints, new_ignores) # Potentially also ignore the original winner and loser

    def resolve_conflict(agent_1_id, agent_2_id, selector):
        pass

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
        self.agents = []
        for i in range(self.num_of_agents):
            newAgent = AgentDistributed(self.my_map, self.starts[i], self.goals[i], i)
            self.agents.append(newAgent)

        
        # Simulation loop
        self.t = 0
        while not all_finished:
            print(f'\n\n\n*****now calculating for time: {self.t}*****')

            # Iterate over all unique combinations of agents
            for i in range(len(self.agents)-1):
                # print(f'agent{i} has recorded path of: {agents[i].path}')

                for j in range(i+1, len(self.agents)):
                    # print(f'\ncomparing planned path of agent {i} and {j}:\nagent{i}: {agents[i].planned_path}\nagent{j}: {agents[j].planned_path}')


                    # Retreieving locations at t and t+1 for both agents in the combination
                    a1_pos1 = self.agents[i].path[-1]
                    a2_pos1 = self.agents[j].path[-1]
                    a1_pos2 = self.agents[i].position_at(self.t+1)
                    a2_pos2 = self.agents[j].position_at(self.t+1)

                    # Generating collision booleans
                    vertex_collided = 1 if a1_pos2 == a2_pos2 else 0
                    edge_collided = 1 if a1_pos1 == a2_pos2 and a2_pos1 == a1_pos2 else 0

                    # Handle collision
                    if vertex_collided or edge_collided:
                        print(f'agents {i}(1{a1_pos1}, 2{a1_pos2}) & {j}(1{a2_pos1}, 2{a2_pos2}) have a collision!\nVertex? {vertex_collided}\nedge? {edge_collided}')
                        

                        # Momentum handling
                        if self.agents[i].momentum != self.agents[j].momentum: 

                            print('detected momentum constrain')
                            winner_id = i if self.agents[i].momentum > self.agents[j].momentum else j
                            loser_id = j if self.agents[i].momentum > self.agents[j].momentum else i

                            loser_constraint = [{'agent': loser_id,
                                                'loc': [self.agents[winner_id].path[-1],self.agents[winner_id].position_at(self.t+1)],
                                                'timestep': self.t}]
                            loser_newplan = self.agents[loser_id].plan_path(loser_constraint, self.t)
                            self.update_others(loser_newplan, loser_constraint, [winner_id, loser_id])
                            # Loser agent is given constraints and has to replan path
                            

                        # If both agents have same momentum
                        else:
                            print('detected momentum constrain w equal momemntum!!!')
                            # Random loser if both agents have same momentum
                            agent_selector = 0 #random.randrange(0, 2) # Random int, either 0 or 1
                            winner_id = i if agent_selector else j
                            loser_id = j if agent_selector else i
                            

                            loser_constraint = [{'agent': loser_id,
                                                'loc': [self.agents[winner_id].position_at(self.t),self.agents[winner_id].position_at(self.t+1)],
                                                'timestep': self.t}]
                            loser_newplan = self.agents[loser_id].plan_path(loser_constraint, self.t)
                            self.update_others(loser_newplan, loser_constraint, [winner_id, loser_id])
            n_finished = 0 
            # print('\nmoving all agents with their current plan.....')
            for agent in self.agents:
                print(f'agent{agent.id} planned:\n{self.agents[agent.id].planned_path}\n has moved from \n{agent.pos} to')
                agent.move_with_plan(self.t)
                print(f'{agent.pos}\nnow it has a recorded path of {agent.path}\n')
                
                n_finished = n_finished + agent.is_on_goal()

            # Ending simulation if all agents are on goals
            if n_finished == len(self.agents): all_finished = True
            # elif t > 5: all_finished = True
            
            print(f'finished for time: {self.t}')
            self.t = self.t + 1
            print(f'moving to time: {self.t}')
            animation_paths = []
            for i in range(len(self.agents)):
                animation_paths.append(self.agents[i].get_last_two_moves(self.t))
            
            # a = Animation(self.my_map, self.starts, self.goals, animation_paths)
            # a.animate_once(animation_paths)
            
        for agent in self.agents:
            result.append(agent.path)
        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.6f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)
        
        return result