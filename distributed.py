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
            print(f'\n\n\n*****now calculating for time: {t}*****')
            # Iterate over all unique combinations of agents
            
            all_locations = []

            for agent in agents:
                all_locations.append(agent.position_at(t + 1))

            for i in range(len(agents)-1):
                # print(f'agent{i} has recorded path of: {agents[i].path}')
                ith_constraints = []
                for k in range(len(all_locations)):
                    ith_constraints = ith_constraints + (constrain_whole_path(all_locations[k], i ,t+1))

                for j in range(i+1, len(agents)):
                    # print(f'\ncomparing planned path of agent {i} and {j}:\nagent{i}: {agents[i].planned_path}\nagent{j}: {agents[j].planned_path}')
                    jth_constraints = []
                    for k in range(len(all_locations)):
                        jth_constraints = jth_constraints + (constrain_whole_path(all_locations[k], j ,t+1))

                    # Retreieving locations at t and t+1 for both agents in the combination
                    a1_pos1 = agents[i].path[-1]
                    a2_pos1 = agents[j].path[-1]
                    a1_pos2 = agents[i].position_at(t+1)
                    a2_pos2 = agents[j].position_at(t+1)
                    
                    if agents[i].moves_back[0] and agents[j].moves_back[0] and (agents[i].moves_back[1] == j or agents[j].moves_back[1] == i): 
                        two_agents = [a1_pos1, a2_pos1]
                        opened_agents = 0

                        for k in range(2):
                            open_cells = 0
                            up = self.my_map[two_agents[k][0]-1][two_agents[k][1]]
                            down = self.my_map[two_agents[k][0]+1][two_agents[k][1]]
                            left = self.my_map[two_agents[k][0]][two_agents[k][1]-1] 
                            right = self.my_map[two_agents[k][0]][two_agents[k][1]+1]
                            directions = [up, down, left, right]

                            for m in directions:
                                if m != False: open_cells +=1

                            if open_cells >= 3:
                                opened_agents += 1

                        if opened_agents == 2:
                            winner_id = i if agents[i].moves_back[0] == "off_goal" else j  # winner is whomever was not on goal
                            loser_id = j if agents[i].moves_back[0] == "off_goal" else i # winner is whomever was on goal

                            winner_path = agents[winner_id].plan_path([], t)
                            loser_constraints = constrain_whole_path(winner_path, loser_id, agents[winner_id].planned_path_t) + jth_constraints if loser_id == j else ith_constraints
                            loser_path = agents[loser_id].plan_path(loser_constraints, t)
                            all_locations.append(agents[i].position_at(t + 1))
                            continue

                    # Generating collision booleans
                    vertex_collided = 1 if a1_pos2 == a2_pos2 else 0
                    edge_collided = 1 if a1_pos1 == a2_pos2 and a2_pos1 == a1_pos2 else 0

                    # Handle collision
                    if vertex_collided or edge_collided:
                        print(f'agents {i}(1{a1_pos1}, 2{a1_pos2}) & {j}(1{a2_pos1}, 2{a2_pos2}) have a collision!\nVertex? {vertex_collided}\nedge? {edge_collided}')
                        # TODO: ADDITIONAL RULES FOR HANDLING GOAL

                        if agents[i].is_on_goal() or agents[j].is_on_goal():
                            # print('                                      detected goal constrain')
                            a1_id = i if agents[j].is_on_goal() else j # Agent 1 needs to pass agent 2
                            a2_id = j if agents[j].is_on_goal() else i # Agent 2 is on goal

                            a2_pos = agents[a2_id].position_at(t)

                            evade_a2_map = copy.deepcopy(self.my_map)
                            evade_a2_map[a2_pos[0]][a2_pos[1]] = True
                            
                            a1_path_evade_a2 = agents[a1_id].try_path(evade_a2_map, jth_constraints if a1_id == j else ith_constraints, t, time_dependent=False)
                            
                            if a1_path_evade_a2: # If a1 was able to replan around a2
                                # print(f'agent {a1_id} can reach goal by going around agent {a2_id}')
                                agents[a1_id].use_path(a1_path_evade_a2, t)
                                # print(f"a1 replans around a2: {a1_path_evade_a2}")

                            else: # If evasion of a2 isnt possible
                                # print(f'agent {a1_id} CANNOT reach goal by going around agent {a2_id}')
                                evade_a1path_constraints = constrain_whole_path(agents[a1_id].planned_path, a2_id, agents[a1_id].planned_path_t) + jth_constraints if a2_id == j else ith_constraints
                                a2_evade_a1path = agents[a2_id].try_path(self.my_map, evade_a1path_constraints, t)

                                if a2_evade_a1path: # Possible for a2 to move out of a1 path
                                    # print(f"a2 replans around a1 path: {a2_evade_a1path}")
                                    agents[a2_id].use_path(a2_evade_a1path, t)
                                    
                                else: # Dead end, both need to move back TODO This currently does not allow moved back agents to replan, they're essentially stuck after they move back
                                    # print(f'agent {a1_id} CANNOT move out of way for agent {a2_id}')
                                    # p1 = copy.deepcopy(agents[a1_id].path)
                                    # p2 = copy.deepcopy(agents[a2_id].planned_path)
                                    # p1.reverse()
                                    # p2.reverse()
                                    p1_constraints = ith_constraints if a1_id == i else jth_constraints
                                    p2_constraints = jth_constraints if a1_id == i else ith_constraints
                                    p1 = agents[a1_id].try_path(self.my_map, p1_constraints, t, goal=agents[a1_id].path[0])
                                    p1 = agents[a2_id].try_path(self.my_map, p2_constraints, t, goal=agents[a2_id].path[0])

                                    # print(f'   a and b look like: {p1}, {p2} ')
                                    agents[a1_id].use_path(p1, t)
                                    agents[a2_id].use_path(p2, t)
                                    agents[a1_id].moves_back = ("off_goal", a2_id)
                                    agents[a2_id].moves_back = ("on_goal", a1_id)


                        # Momentum handling
                        elif agents[i].momentum != agents[j].momentum: 
                            print('detected momentum constrain')
                            winner_id = i if agents[i].momentum > agents[j].momentum else j
                            loser_id = j if agents[i].momentum > agents[j].momentum else i

                            # Loser agent is given constraints and has to replan path
                            loser_constraints = constrain_whole_path(agents[winner_id].planned_path, loser_id, agents[winner_id].planned_path_t) + jth_constraints if loser_id == j else ith_constraints
                            agents[loser_id].plan_path(loser_constraints, t)
                            # print(f'aa agent{loser_id} planned a new path: {agents[loser_id].planned_path}')

                        else:
                            print('detected momentum constrain w equal momemntum!!!')
                            # Random loser if both agents have same momentum
                            agent_selector = 0 #random.randrange(0, 2) # Random int, either 0 or 1
                            winner_id = i if agent_selector else j
                            loser_id = j if agent_selector else i
                            
                            loser_constraints = constrain_whole_path(agents[winner_id].planned_path, loser_id, agents[winner_id].planned_path_t) + jth_constraints if loser_id == j else ith_constraints
                            agents[loser_id].plan_path(loser_constraints, t)
                            # print(f'cc agent{loser_id} planned a new path: {agents[loser_id].planned_path}')
                            
                all_locations.append(agents[i].position_at(t + 1))


            n_finished = 0 
            # print('\nmoving all agents with their current plan.....')
            for agent in agents:
                # print(f'agent{agent.id} planned:\n{agents[agent.id].planned_path}\n has moved from \n{agent.pos} to')
                agent.move_with_plan(t)
                # print(f'{agent.pos}\nnow it has a recorded path of {agent.path}\n')
                
                n_finished = n_finished + agent.is_on_goal()

            # Ending simulation if all agents are on goals
            if n_finished == len(agents): all_finished = True
            # elif t > 5: all_finished = True
            
            print(f'finished for time: {t}')
            t = t + 1
            print(f'moving to time: {t}')
            animation_paths = []
            for i in range(len(agents)):
                animation_paths.append(agents[i].get_last_two_moves(t))
            
            # a = Animation(self.my_map, self.starts, self.goals, animation_paths)
            # a.animate_once(animation_paths)
            
        for agent in agents:
            result.append(agent.path)
        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.6f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)
        
        return result