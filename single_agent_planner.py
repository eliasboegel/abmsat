import heapq
from operator import truth

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values




def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path



def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def node_constructor(loc, g_val, h_val, time_step, parent):
    node = {'loc': loc,
            'g_val': g_val,
            'h_val': h_val,
            'timestep': time_step,
            'parent': parent}
    return node


def is_constrained(curr_loc, child_loc, next_time, constraint_table): # badly written, need to see if can be improved
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    constrained = 0
    blocked_loc = (0,0)

    edge_key = (child_loc, next_time-1) # key for checking if edge is blocked
    vertex_key = (child_loc, next_time) # key for checking if vertex is blocked
    if edge_key in constraint_table:
        blocked_loc = constraint_table[edge_key]['loc'][0]
        constrained = 'stand'
    elif vertex_key in constraint_table:
        blocked_loc = constraint_table[vertex_key]['loc'][0]
        constrained = 'move'
            
    return constrained, blocked_loc


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    table = dict()
    for i in constraints:
        if i['agent'] == agent:
            smaller_constraint = {'loc': i['loc'],
                                  'timestep': i['timestep']}
            constraint_key = (i['loc'][0], i['timestep'])
            table[constraint_key] = smaller_constraint
    return table


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    open_list = []
    closed_list = dict()
    constraint_table = build_constraint_table(constraints, agent)
    print(f'constraint table: {constraint_table}')

    # setting up root node and adding it to open and closed list, to initiate the algorithm
    h_value = h_values[start_loc]
    root = {'loc': start_loc,
        'g_val': 0,
        'h_val': h_value,
        'timestep': 0,
        'parent': None}
    push_node(open_list, root)
    closed_list[(root['loc'], 0)] = root

    dims = (len(my_map), len(my_map[0]))

    while len(open_list) > 0:
        curr = pop_node(open_list)
        curr_loc = curr['loc']
        curr_time = curr['timestep']
        child_time = curr['timestep'] + 1

        print(f'moved to {curr_loc}')
        print(f'\n\ncurrently at {curr_loc} at time {curr_time}')

        curr_cost = curr['g_val']+curr['h_val']
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr_loc == goal_loc: # if the current location is the goal location then return the path
            return get_path(curr)

        for dir in range(5): # exploring all five moves that can be taken from the current location
            child_loc = move(curr['loc'], dir)
            print(f'\nChecking for direction: {move((0,0), dir)} (moving to {child_loc})')
            
            if -1<child_loc[0]<dims[0] and -1<child_loc[1]<dims[1]: # checking if the child location is within the map
                if my_map[child_loc[0]][child_loc[1]]: # if the child location is an obstacle then skip it
                    print(f'child location has an obstacle')
                    continue
                
                child_cost = curr['g_val'] + 1 + h_values[child_loc]
                print(f'current cost vs child cost: {curr_cost} vs {child_cost}')
                
                constrained, blocked_loc = is_constrained(curr_loc, child_loc, child_time, constraint_table)
                print(f'direction constrained?: {constrained}, blocked cell is {blocked_loc}')
                

                # checking if direction is constrained, and changing the type of child node accordingly
                if constrained == 'move':
                    if child_loc == blocked_loc:
                        print(f'cannot move to child location')
                        continue
                    child = {'loc': child_loc,
                             'g_val': curr['g_val'] +1,
                             'h_val': h_values[child_loc],
                             'timestep': child_time,
                             'parent': curr}
                elif constrained == 'stand':
                    if child_loc == blocked_loc:
                        print(f'cannot stand in child location')
                        continue
                    child = {'loc': child_loc,
                             'g_val': curr['g_val'] +1,
                             'h_val': h_values[child_loc],
                             'timestep': child_time,
                             'parent': curr}
                else:                
                    child = {'loc': child_loc,
                        'g_val': curr['g_val'] + 1,
                        'h_val': h_values[child_loc],
                        'timestep': child_time,
                        'parent': curr}

                child_key = (child['loc'], child['timestep'])
                    
                if (child_key) in closed_list: 
                    existing_node = closed_list[child_key]
                    if compare_nodes(child, existing_node):
                        closed_list[child_key] = child
                        push_node(open_list, child)
                else:
                    closed_list[child_key] = child
                    push_node(open_list, child)
            else:
                print(f'direction {move((0,0), dir)} goes out of map ({child_loc})')
                pass
    return None  # Failed to find solutions
