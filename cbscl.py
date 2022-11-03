import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, get_path


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    
    # Iterate over all times as long as at least one agent has not reached the goal yet
    #print(len(path1))
    #print(len(path2))
    for t in range(max(len(path1), len(path2))):

        # Extract positions of both agents
        pos1 = get_location(path1, t)
        pos1_next = get_location(path1, t+1)
        pos2 = get_location(path2, t)
        pos2_next = get_location(path2, t+1)

        if pos1_next == pos2_next:
            # Return vertex collision if both agents are in the same location
            return {'loc': [pos1_next, pos1_next], 'timestep': t}
        elif pos1 == pos2_next and pos1_next == pos2:
            # Return edge collision if agents are in axis-aligned (not diagonal) adjacent locations
            return {'loc': [pos2, pos1], 'timestep': t}

    # Return None if no collision was found for any timestep
    return None

def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    
    # Initialize empty list to store collisions in
    collisions = []

    # Iterate over all unique combinations of collisions
    for i in range(len(paths) - 1):
        for j in range(i + 1, len(paths)):

            # Detect collision between two individual agents
            collision = detect_collision(paths[i], paths[j])
            
            # If collision was detected, store collision in list
            if collision is not None:
                collision['a1'] = i
                collision['a2'] = j
                collisions.append(collision)

    return collisions

def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    
    # Create constraint for agent1
    constraint1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']}
    
    # Create constraint for agent2, if edge constraint -> reverse location order
    constraint2 = {'agent': collision['a2'], 'loc': collision['loc'][::-1], 'timestep': collision['timestep']}
    
    # Return complete list of constraints
    return [constraint1, constraint2]


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    
    # Randomly select one agent as "agent1" and the other as "agent2"
    agent_selector = random.randrange(0, 2)
    agent1_local_id = agent_selector # Local id, i.e. first or second agent within this collision only
    agent2_local_id = 0 if agent1_local_id else 1 # If agent1 is 0, then agent2 is 1 and vice-versa
    agent1_global_id = collision['a1'] if agent_selector else collision['a2'] # Id of agent1 out of all agents
    agent2_global_id = collision['a2'] if agent_selector else collision['a1'] # Id of agent2 out of all agents
    
    # Force move away from current location
    constraints1 = [{'agent': agent1_global_id, 'loc': collision['loc'][agent1_local_id], 'timestep': collision['timestep']}]
    #print(f"agent1: {agent1_local_id}, {agent1_global_id}, {collision['loc'][agent1_local_id]}")
    #print(f"agent2: {agent2_local_id}, {agent2_global_id}, {collision['loc'][agent2_local_id]}")
    # Constrain all directions but the one in which agent1 is forced into
    for i in range(-1, 2):
        for j in range(-1, 2):

             # Filter out diagonals
            if abs(i) == abs(j): continue

            #print(f"col loc: {collision['loc']}")
            loc = (collision['loc'][agent1_local_id][0] + i, collision['loc'][agent1_local_id][1] + j)   
            if loc != collision['loc'][agent2_local_id]:
                constraints1.append({'agent': agent1_global_id, 'loc': loc, 'timestep': collision['timestep']})
    
    # Prevent agent2 from moving over the edge or into the vertex into which agent1 moves
    constraint2 = [{'agent': agent2_global_id, 'loc': collision['loc'][::-1 if agent2_local_id else 1], 'timestep': collision['timestep']}]
    
    #print(f"constraints: {constraints1 + constraint2}")
    # Return complete list of constraints
    return constraints1 + constraint2
    

class CBSCLSolver(object):
    """The high-level search of CBS, cycle limited."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=False, limited_cycle_length=1):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [[]],
                'collisions': [],
                'history': []}

        # Find initial path for each agent
        for i in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'][0].append(path)

        root['cost'] = get_sum_of_cost(root['paths'][-1])
        root['collisions'] = detect_collisions(root['paths'][-1])
        self.push_node(root)

        # Task 3.1: Testing
        # print(f"Collisions: {root['collisions']}")

        # Task 3.2: Testing
        #for collision in root['collisions']:
        # print(f"Constraints: {standard_splitting(collision)}")
        
        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        # While open nodes still exist
        while self.open_list:
            print(len(self.open_list))
            
            # Retrieve open node and remove it from list
            p = self.pop_node()
            #print(f"Original paths: {p['paths']}")

            # Detect collisions between all agents
            p['collisions'] = detect_collisions(p['paths'][-1])
            #print(f"pre-replan path: {p['paths']}")
            #print(f"#collisions: { len(p['collisions'])}")
            #print(f"last collision: {p['collisions']}")

            # Return current paths if no collisions were detected between them
            if not p['collisions']: return p['paths'][-1]

            # Select one arbitrary (in this case first) collision from all detected collisions
            collision = p['collisions'][0]

            # Generate constraints for the two involved agents based on the selected collisions
            constraints = disjoint_splitting(collision) if disjoint else standard_splitting(collision)
            #print(f"new constraints: {constraints}")



            # Iterate through all constraints generated
            #agent_selector = random.randrange(0, 2)
            for new_constraint in constraints: #[constraints[agent_selector]]:
                # Create new blank node with the newly generated constraints and previous paths
                q = {
                    'constraints': p['constraints'].copy() + [new_constraint]
                }

                

                # Retrieve index of agent for which the current constraint was generated
                agent = new_constraint['agent']
                
                #print(f"New constraint: {new_constraint}")
                #print(f"before: {q['paths'][agent]}")
                # Generate new path using the new additional constraints (i.e. avoiding the collision)
                new_path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent, q['constraints'])
                #print(f"after: {path}")
                # If a path was found, push the new node with updated path back to the open list

                # Second part of conditional limits 1-timestep cycles
                if (new_path is not None):

                    # Cycle limiting
                    min_repeats_to_assume_cycle = 2 # Number of repeating sequences needed before node is deemed cycling indefinitely, lower numbers limit more agressively, but also more incorrectly
                    cycle_detected = False
                    test_paths = p['paths'].copy()
                    test_paths.append(p['paths'][-1].copy())
                    test_paths[-1][agent] = new_path.copy()
                    for cycle_length in range(1, len(test_paths) // (min_repeats_to_assume_cycle+1)): # Max cycle length is the maximum length that can fit at least twice into the path history
                        all_same = True
                        for i in range(min_repeats_to_assume_cycle):
                            last_n_history = test_paths[-cycle_length:] # Path history for all agents for last n nodes of the parent nodes, where n = cycle length
                            n_history_before_last = test_paths[-(min_repeats_to_assume_cycle+1)*cycle_length:-min_repeats_to_assume_cycle*cycle_length] # Path history for all agents for set of n parent nodes before the last n parent nodes
                            
                            if last_n_history != n_history_before_last:
                                all_same = False

                        if all_same:
                            cycle_detected = True
                            print(f"Cycle of length {cycle_length} detected, closing node!")
                            break

                    if not cycle_detected:
                        q['paths'] = test_paths
                        q['history'] = p['history'].copy() + [new_path.copy()]
                        q['collisions'] = detect_collisions(q['paths'][-1])
                        q['cost'] = get_sum_of_cost(q['paths'][-1])
                        #print(f"Same path as previous node: {p['paths'][agent] == q['paths'][agent]}\n")
                        self.push_node(q)
                        #print(f"q path: {q['paths']}")
                #else:
                    #print("Solution not found")
            #print( '------------------------------')


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))