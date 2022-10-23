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
        self.start_time = time.time()


    def update_others(self, loser_newplan, init_constraints, ignored_ids, invoker_id, reverse=False):
        """Updates the paths of all agents except the ones in ignored_ids"""
        print(f'triggered {invoker_id}')
        constraints = init_constraints
        print(f'çonstrsins {constraints}')

        print(f"ignored: {ignored_ids}")
        visible_agents = []
        for combo in self.radar_combos:
            if combo[0] == invoker_id and combo[0] not in ignored_ids:
                visible_agents.append(combo[1])
            elif combo[1] == invoker_id and combo[1] not in ignored_ids:
                visible_agents.append(combo[0])
                
        # Update other agents in descending momentum priority order
        sorted_visible_agents = sorted(visible_agents, key=lambda agent_id: self.agents[agent_id].momentum, reverse=True)
        for visible_agent in sorted_visible_agents:
            print(f'vis agent: {visible_agent}')
            next_constraints = constraints + constrain_path([loser_newplan[0]], visible_agent, self.t)
            # constraints = constraints + constrain_path([loser_newplan[0]] + constrained_pos, other_agent_id, self.t, dt=0)
            # filtered_constraints = list(filter(lambda constraint: constraint["agent"] != other_agent_id, constraints))
            new_path = self.agents[visible_agent].plan_path(next_constraints, self.t, goal=self.agents[visible_agent].start if reverse else self.agents[visible_agent].goal)
            new_ignores = [visible_agent]
            # print(f'new ignoressss {new_ignores}')
            
            self.update_others(new_path, next_constraints, new_ignores, visible_agent) # Potentially also ignore the original winner and loser


    def resolve_conflict(self, a1_id, a2_id, selector, reverse=False, used_map=None):
        """Resolves a conflict between two agents by selecting a winner and a loser"""
        used_map = self.my_map if used_map is None else used_map

        winner_id = a1_id if selector else a2_id
        loser_id = a2_id if selector else a1_id

        loser_constraint = [{'agent': loser_id,
                            'loc': [self.agents[winner_id].path[-1],self.agents[winner_id].position_at(self.t+1)],
                            'timestep': self.t}]
        self.agents[loser_id].curr_constraints = self.agents[loser_id].curr_constraints + loser_constraint
        loser_newplan = self.agents[loser_id].plan_path(self.agents[loser_id].curr_constraints, self.t, self.agents[loser_id].start if reverse else self.agents[loser_id].goal, used_map=used_map)
        print(f"loser new plan: {loser_newplan}")

        # Loser is given constraints and has to replan path
        self.update_others(loser_newplan, loser_constraint, [loser_id], loser_id, reverse)


    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations.
        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants       
        fix, ax = plt.subplots()
        start_time = time.time()
        result = []
        all_finished = False
        dims = (len(self.my_map), len(self.my_map[0]))
        self.CPU_time = time.time() - start_time
        
        
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

            self.radar_combos = []
            for id_1 in range(len(self.agents)-1):
                for id_2 in range(id_1+1, len(self.agents)):
                    rel_x = self.agents[id_1].path[-1][0] - self.agents[id_2].path[-1][0]
                    rel_y = self.agents[id_1].path[-1][1] - self.agents[id_2].path[-1][1]

                    if (abs(rel_x) + abs(rel_y)) < 3:
                        self.radar_combos.append((id_1,id_2))
                        
            # print(f"Mut comb: {self.radar_combos}")


            for combination in self.radar_combos:
                i = combination[0]
                j = combination[1]

                # Retreieving locations at t and t+1 for both agents in the combination
                a1_pos1 = self.agents[i].path[-1]
                a2_pos1 = self.agents[j].path[-1]
                a1_pos2 = self.agents[i].position_at(self.t+1)
                a2_pos2 = self.agents[j].position_at(self.t+1)

                if self.agents[i].moves_back[0] and self.agents[j].moves_back[0] and (self.agents[i].moves_back[1] == j or self.agents[j].moves_back[1] == i): 
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
                        selector = self.agents[i].moves_back[0] == "off_goal"
                        winner_id = i if selector else j
                        loser_id = j if selector else i

                        # winner_id = i if self.agents[i].moves_back[0] == "off_goal" else j  # winner is whomever was not on goal
                        # loser_id = j if self.agents[i].moves_back[0] == "off_goal" else i # winner is whomever was on goal
                        
                        # need to update everyone here
                        winner_path = self.agents[winner_id].plan_path([], self.t)
                        loser_constraint = constrain_path(winner_path, loser_id, self.agents[winner_id].planned_path_t)
                        loser_path = self.agents[loser_id].plan_path(loser_constraint, self.t)

                        self.update_others(loser_path, loser_constraint, [winner_id, loser_id], loser_id, reverse=False)
                        continue


                # Generating collision booleans
                vertex_collided = 1 if a1_pos2 == a2_pos2 else 0
                edge_collided = 1 if a1_pos1 == a2_pos2 and a2_pos1 == a1_pos2 else 0

                # Handle collision
                if vertex_collided or edge_collided:
                    print(f'agents {i} (1{a1_pos1}, 2{a1_pos2}) & {j} (1{a2_pos1}, 2{a2_pos2}) have a collision!\nVertex? {vertex_collided}\nedge? {edge_collided}')
                    
                    if self.agents[i].is_on_goal() or self.agents[j].is_on_goal():
                        # print("Goal constraint!")
                        off_goal_id = i if self.agents[j].is_on_goal() else j # Agent 1 needs to pass agent 2
                        on_goal_id = j if self.agents[j].is_on_goal() else i # Agent 2 is on goal

                        off_goal_pos = self.agents[off_goal_id].position_at(self.t)
                        on_goal_pos = self.agents[on_goal_id].position_at(self.t)

                        self.agents[off_goal_id].obstructed_map = copy.deepcopy(self.agents[off_goal_id].obstructed_map)
                        self.agents[off_goal_id].obstructed_map[on_goal_pos[0]][on_goal_pos[1]] = True
                        
                        path_around_agent_on_goal = self.agents[off_goal_id].try_path(self.agents[off_goal_id].obstructed_map, self.agents[off_goal_id].curr_constraints, self.t, time_dependent=False)
                        
                        if path_around_agent_on_goal: # If off_goal du was able to replan around a2
                            print(f"Replanning {off_goal_id} around {on_goal_id}")
                            #self.agents[off_goal_id].use_path(path_around_agent_on_goal, self.t)
                            #self.update_others(path_around_agent_on_goal, self.agents[off_goal_id].curr_constraints, [on_goal_id, off_goal_id], off_goal_id)
                            self.resolve_conflict(off_goal_id, on_goal_id, 0, used_map=self.agents[off_goal_id].obstructed_map)

                        else: # Dead end, both need to move back TODO This currently does not allow moved back agents to replan, they're essentially stuck after they move back
                            print(f"Moving back {off_goal_id}")
                            # print(f'agent {off_goal_id} CANNOT go around agent {on_goal_id}')
                            self.agents[off_goal_id].moves_back = ("off_goal", on_goal_id)
                            self.agents[on_goal_id].moves_back = ("on_goal", off_goal_id)
                            self.resolve_conflict(on_goal_id, off_goal_id, 0, reverse=True)

    
                    # Momentum handling
                    elif self.agents[i].momentum != self.agents[j].momentum: 
                        print("Unequal momentum!")
                        higher_momentum = self.agents[i].momentum > self.agents[j].momentum
                        self.resolve_conflict(i, j, higher_momentum)
                        
                    # If both agents have same momentum
                    else:
                        print('Equal momentum')
                        # Random loser if both agents have same momentum
                        select_random = 0#random.randrange(0, 2) # Random int, either 0 or 1
                        self.resolve_conflict(i, j, select_random)
            
            n_finished = 0
            for agent in self.agents:
                print(f'agent{agent.id} planned (i have goal = {agent.goal}):{self.agents[agent.id].planned_path}\n has moved from \n{agent.pos} to')
                agent.move_with_plan(self.t)
                print(f'{agent.pos}\nnow it has a recorded path of {agent.path}\n')
                
                n_finished = n_finished + agent.is_on_goal()

            # Ending simulation if all agents are on goals
            if n_finished == len(self.agents): all_finished = True
            elif self.t > 22: all_finished = True
            
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
        print(f"time.time diff: {round(time.time() - self.start_time, 6)}")
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)
        
        return result