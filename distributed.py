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
        """Updates the paths of all agents except the ones in ignored_ids"""
        constraints = init_constraints
        
        # Creating the other_agents list
        other_agents = list(filter(lambda id: id not in ignored_ids, [*range(len(self.agents))]))

        # Sort 
        sorted_other_agent_ids = sorted(other_agents, key=lambda agent_id: self.agents[agent_id].momentum, reverse=True)
        # Update other agents in descending momentum priority order
        print(f'sorted other agents list: {sorted_other_agent_ids}')
        for other_agent_id in sorted_other_agent_ids:
            constraints = constraints + constrain_path([loser_newplan[0]], other_agent_id, self.t)
            new_path = self.agents[other_agent_id].plan_path(constraints, self.t)
            new_ignores = [other_agent_id] + ignored_ids
            print(f'new ignoressss {new_ignores}')
            
            self.update_others(new_path, constraints, new_ignores) # Potentially also ignore the original winner and loser


    def resolve_conflict(self, a1_id, a2_id, selector):
        """Resolves a conflict between two agents by selecting a winner and a loser"""

        winner_id = a1_id if selector else a2_id
        loser_id = a2_id if selector else a1_id

        loser_constraint = [{'agent': loser_id,
                            'loc': [self.agents[winner_id].path[-1],self.agents[winner_id].position_at(self.t+1)],
                            'timestep': self.t}]
        loser_newplan = self.agents[loser_id].plan_path(loser_constraint, self.t)

        # Loser is given constraints and has to replan path
        self.update_others(loser_newplan, loser_constraint, [winner_id, loser_id])


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
            all_positions = []
            for agent in self.agents:
                all_positions.append(agent.position_at(self.t))

            print(f'\n*****now calculating for time: {self.t}*****')

            mutually_visible_combinations = []
            for id_1 in range(len(self.agents)-1):
                for id_2 in range(i+1, len(self.agents)):
                    rel_x = self.agents[i].path[-1][0] - self.agents[i].path[-1][0]
                    rel_y = self.agents[i].path[-1][1] - self.agents[i].path[-1][1]

                    if abs(rel_x) < 3 and abs(rel_y) < 3:
                        mutually_visible_combinations.append((id_1,id_2))

                    # for x in range(-2, 3):
                    #     for y in range(-2, 3):
                    #         curr_pos = self.agents[i].path[-1]
                    #         test_pos = (curr_pos[0] + x, curr_pos[1] + y)
                    #         if test_pos in all_positions:
                    #             visible_agent_ids.append(all_positions.index(test_pos))

            

            # Iterate over all unique combinations of agents

            for i in range(len(self.agents)-1):
                visible_agent_ids = []
                for x in range(-2, 3):
                    for y in range(-2, 3):
                        curr_pos = self.agents[i].path[-1]
                        test_pos = (curr_pos[0] + x, curr_pos[1] + y)
                        if test_pos in all_positions:
                            visible_agent_ids.append(all_positions.index(test_pos))

                print(f"agent {i}")
                for j in visible_agent_ids:
                    print(f"visible agents: {j}")
                
                    pass
                
                for j in range(i+1, len(self.agents)):
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
                        print(f'agents {i} (1{a1_pos1}, 2{a1_pos2}) & {j} (1{a2_pos1}, 2{a2_pos2}) have a collision!\nVertex? {vertex_collided}\nedge? {edge_collided}')
                        
                        if self.agents[i].is_on_goal() or self.agents[j].is_on_goal():
                            print("Goal constraint!")
                            off_goal_id = i if self.agents[j].is_on_goal() else j # Agent 1 needs to pass agent 2
                            on_goal_id = j if self.agents[j].is_on_goal() else i # Agent 2 is on goal

                            off_goal_pos = self.agents[off_goal_id].position_at(self.t)
                            on_goal_pos = self.agents[on_goal_id].position_at(self.t)

                            evade_agent_on_goal_map = copy.deepcopy(self.my_map)
                            evade_agent_on_goal_map[on_goal_pos[0]][on_goal_pos[1]] = True
                            
                            path_around_agent_on_goal = self.agents[off_goal_id].try_path(evade_agent_on_goal_map, [], self.t, time_dependent=False)
                            
                            if path_around_agent_on_goal: # If off_goal du was able to replan around a2
                                self.agents[off_goal_id].use_path(path_around_agent_on_goal, self.t)
                                self.update_others(path_around_agent_on_goal, [], [on_goal_id, off_goal_id])

                        
                        # Momentum handling
                        elif self.agents[i].momentum != self.agents[j].momentum: 
                            print("Unequal momentum!")
                            higher_momentum = self.agents[i].momentum > self.agents[j].momentum
                            self.resolve_conflict(i, j, higher_momentum)
                            
                        # If both agents have same momentum
                        else:
                            print('Equal momentum')
                            # Random loser if both agents have same momentum
                            select_random = random.randrange(0, 2) # Random int, either 0 or 1
                            self.resolve_conflict(i, j, select_random)
            
            n_finished = 0
            for agent in self.agents:
                print(f'agent{agent.id} planned (goal = {agent.goal}):\n{self.agents[agent.id].planned_path}\n has moved from \n{agent.pos} to')
                agent.move_with_plan(self.t)
                print(f'{agent.pos}\nnow it has a recorded path of {agent.path}\n')
                
                n_finished = n_finished + agent.is_on_goal()

            # Ending simulation if all agents are on goals
            if n_finished == len(self.agents): all_finished = True
            elif self.t > 20: all_finished = True
            
            print(f'finished for time: {self.t}')
            self.t = self.t + 1
            print(f'\n\n\n*****     *****moving to time: {self.t}*****     *****')
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