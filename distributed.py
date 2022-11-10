"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

from math import comb
import time, random, copy
from agent import AgentDistributed
from visualize import Animation
import matplotlib.pyplot as plt
from single_agent_planner import get_sum_of_cost, constrain_path
import numpy as np

np.random.seed(1)
# ISSUES THAT NEED TO BE FIXED
# 1) Vertex/edge collisions with agents outside of the current combination are not checked properly.
#    This causes problems when resolve_conflict is called and a vertex/edge conflict occurs directly for the first move
#    (A part of the) Solution should be to include constraints of other observed agents with their t+1 positions in the replanning for resolve_conflict

# 2) A special case must be treated:
#    Consider the following map (like map 9)
#    Initial positions
#    @ @ @ @ @ @ @ @ @ @
#    @ @ @ @ @         @
#    @(a)       (b)    @
#    @ @ @ @ @ @ @ @ @ @
#    Final positions
#    @ @ @ @ @ @ @ @ @ @
#    @ @ @ @ @         @
#    @ [b]      [a]    @
#    @ @ @ @ @ @ @ @ @ @
#    What happens is:
#    @ @ @ @ @ @ @ @ @ @
#    @ @ @ @ @         @
#    @ a b             @
#    @ @ @ @ @ @ @ @ @ @
#    Now, a has nowhere to go, but neither are on the goal, and there is no way to avoid each other
#    Correct behaviour: both must enter the backtracking mode until there is sufficient space to switch
#    Actual behaviour: A crash upon a not being able to find a solution

class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals, heuristics_func=None):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.start_time = time.time()
        self.heuristic_func = 'old' if heuristics_func is None else heuristics_func
        self.collision_constraints = []


    def update_others(self, invoker_newplan, init_constraints, ignored_ids, invoker_id, reverse=False):
        """Updates the paths of all agents except the ones in ignored_ids"""
        # print(f'update others invoked by: {invoker_id}')
        # Generate list of visible agents
        visible_agents = []
        for combo in self.radar_combos:
            if combo[0] == invoker_id and combo[1] not in ignored_ids:
                visible_agents.append(combo[1])
            elif combo[1] == invoker_id and combo[0] not in ignored_ids:
                visible_agents.append(combo[0])
        
        # Update other agents in descending momentum priority order
        sorted_visible_agents = sorted(visible_agents, key=lambda agent_id: self.agents[agent_id].momentum, reverse=True)
        # print(f'visible agents: {sorted_visible_agents}')
        for visible_agent in sorted_visible_agents:

            # Make constraints valid for current agent
            constraints = init_constraints
            for i in range(len(constraints)):
                constraints[i]['agent'] = visible_agent
            

            new_ignores = [visible_agent] + ignored_ids
            # new_ignores = [visible_agent]
            # new_ignores = []
            # print(f'\nignored_ids from invoker: {ignored_ids}\ninvoker newplan (agent{invoker_id}): {invoker_newplan}\nvisible agent (agent{visible_agent}) planned path: {self.agents[visible_agent].planned_path}')
            collision = self.check_collision(invoker_id, visible_agent)
            if collision:
                # collision_constraints = constraints + constrain_path([collision, loser_newplan[0]], visible_agent, self.t, dt=0)
                # collision_constraints = constraints + [{'agent': visible_agent,
                #                                         'loc': [collision, collision],
                #                                         'timestep': self.t},
                #                                         {'agent': visible_agent,
                #                                         'loc': [invoker_newplan[0], invoker_newplan[0]],
                #                                         'timestep': self.t}]
                # print(f'collision between {invoker_id} and {visible_agent} at {collision}')
                if collision == invoker_newplan[0]:
                    collision_constraints = constraints + [{'agent': visible_agent,
                                                        'loc': [invoker_newplan[0]],
                                                        'timestep': self.t}]
                else:
                    collision_constraints = constraints + [{'agent': visible_agent,
                                                        'loc': [invoker_newplan[0], invoker_newplan[1]],
                                                        'timestep': self.t}]
                next_path = self.agents[visible_agent].plan_path(collision_constraints, self.t, goal=self.agents[visible_agent].start if reverse else self.agents[visible_agent].goal)
                self.update_others(next_path, collision_constraints, new_ignores, visible_agent) # Potentially also ignore the original winner and loser


    def resolve_conflict(self, a1_id, a2_id, selector, reverse=False, used_map=None):
        """Resolves a conflict between two agents by selecting a winner and a loser"""
        used_map = self.my_map if used_map is None else used_map

        winner_id = a1_id if selector else a2_id
        loser_id = a2_id if selector else a1_id

        loser_constraint = [{'agent': loser_id,
                            'loc': [self.agents[winner_id].path[-1],self.agents[winner_id].position_at(self.t+1)],
                            'timestep': self.t}]
        self.agents[loser_id].curr_constraints = self.agents[loser_id].curr_constraints + loser_constraint
        
        # ISSUE IS HERE
        # EVEN THO WE CALL UPDATE, THE PLAN PATH OVERWRITES ANY PREVIOUS UPDATE IN THE SAME TIMESTEP
        
        loser_newplan = self.agents[loser_id].plan_path(self.agents[loser_id].curr_constraints, self.t, self.agents[loser_id].start if reverse else self.agents[loser_id].goal, used_map=used_map)

        # Loser is given constraints and has to replan path
        self.update_others(loser_newplan, loser_constraint, [winner_id], loser_id, reverse)

    def check_collision(self, agent1_id, agent2_id):
        # Checks two agents for collisions
        # Returns None if there is no collision
        # Returns the collision position if there is a collision

        a1_pos1 = self.agents[agent1_id].path[-1]
        a2_pos1 = self.agents[agent2_id].path[-1]
        # print(f'invoker is at {a1_pos1} and visible agent is at {a2_pos1}')
        a1_pos2 = self.agents[agent1_id].position_at(self.t+1)
        a2_pos2 = self.agents[agent2_id].position_at(self.t+1)

        vertex_collided = 1 if a1_pos2 == a2_pos2 else 0
        edge_collided = 1 if a1_pos1 == a2_pos2 and a2_pos1 == a1_pos2 else 0
        collided = vertex_collided or edge_collided

        return a1_pos2 if collided else None

    
    def find_wall_costs(self, cell, my_map):
        cost = 0
        x, y = cell[0], cell[1]
        for i in range(len(my_map)):
            for j in range(len(my_map[0])):
                if my_map[i][j]:
                    cost += 3*np.exp(-2*np.sqrt((x-i)**2 + (y-j)**2))
        return cost

    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations.
        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants
        start_time = time.time()
        result = []
        all_finished = False
        self.CPU_time = time.time() - start_time
                
        
        # wall_cost = 0
        nx = len(self.my_map)
        ny = len(self.my_map[0])
        wall_cost = np.zeros((nx, ny))
        goals = None
        if self.heuristic_func == 'potential':
            for i in range(len(self.my_map)):
                for j in range(len(self.my_map[0])):
                    if self.my_map[i][j] == False:
                        wall_cost[i][j] = self.find_wall_costs((i,j), self.my_map)
        elif self.heuristic_func == 'goals':
            goals = self.goals

        # Create list of agent objects with AgentDistributed class
        self.agents = []
        for i in range(self.num_of_agents):
            newAgent = AgentDistributed(self.my_map, self.starts[i], self.goals[i], i, wall_cost, self.heuristic_func, goals)
            self.agents.append(newAgent)

        
        # Simulation loop
        self.t = 0
        while not all_finished:
            all_positions = []
            # print(f'\n*****now calculating for time: {self.t}*****')
            for agent in self.agents:
                all_positions.append(agent.position_at(self.t))

            self.radar_combos = []
            for id_1 in range(len(self.agents)-1):
                for id_2 in range(id_1+1, len(self.agents)):
                    rel_x = self.agents[id_1].path[-1][0] - self.agents[id_2].path[-1][0]
                    rel_y = self.agents[id_1].path[-1][1] - self.agents[id_2].path[-1][1]

                    if (abs(rel_x) + abs(rel_y)) < 3:
                        self.radar_combos.append((id_1,id_2))

            for combination in self.radar_combos:

                i = combination[0]
                j = combination[1]

                # # Retreieving locations at t and t+1 for both agents in the combination
                a1_pos1 = self.agents[i].path[-1]
                a2_pos1 = self.agents[j].path[-1]
                a1_pos2 = self.agents[i].position_at(self.t+1)
                a2_pos2 = self.agents[j].position_at(self.t+1)

                # Handle collision
                if self.check_collision(i, j):
                    # print(f'\n\nagents {i} (1{a1_pos1}, 2{a1_pos2}) & {j} (1{a2_pos1}, 2{a2_pos2}) have a collision!')#\nVertex? {vertex_collided}\nedge? {edge_collided}')
                    # Momentum handling
                    if self.agents[i].momentum != self.agents[j].momentum: 
                        # print("Unequal momentum!")
                        higher_momentum = self.agents[i].momentum > self.agents[j].momentum
                        self.resolve_conflict(i, j, higher_momentum)
                    # If both agents have same momentum
                    else:                        
                        # print('Equal momentum')
                        # Random loser if both agents have same momentum
                        select_random = random.randrange(0, 2) # Random int, either 0 or 1
                        self.resolve_conflict(i, j, select_random)
            
            n_finished = 0
            for agent in self.agents:
                agent.move_with_plan(self.t)
                n_finished = n_finished + agent.is_on_goal()

            # Ending simulation if all agents are on goals
            if n_finished == len(self.agents): all_finished = True
            elif self.t > 9*22: all_finished = True
            
            self.t = self.t + 1
            # print(f'\n\n\n*****     *****moving to time: {self.t}*****     *****')
            self.collision_constraints = []

            # animation_paths = []
            # for i in range(len(self.agents)):
            #     animation_paths.append(self.agents[i].get_last_two_moves(self.t))
            
        for agent in self.agents:
            result.append(agent.path)
        
        # Print final output
        # print("\n Found a solution! \n")
        # print("CPU time (s):    {:.6f}".format(self.CPU_time))
        # print(f"time.time diff: {round(time.time() - self.start_time, 6)}")
        # print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        # print(result)
        
        return result
