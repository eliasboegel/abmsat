import heapq
import numpy as np

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def get_trip_duration(paths):
    durations = 0
    for path in paths:
        goal = path[-1]
        index = -1
        while goal == path[index]:
            index -= 1
        duration = len(path) + index + 1
        durations += duration
    return durations

def get_trip_length(paths):
    length = 0
    for path in paths:
        for i in range(len(path)-1):
            loc1, loc2 = path[i], path[i+1]
            if loc1 != loc2:
                length += 1            
    return length

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
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def compute_heuristics_goals(my_map, goal, goals):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location, but now adding additional cost for goal cells
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
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        additional_cost = 0
        if loc in goals and loc != goal:
            additional_cost = 20
        h_values[loc] = node['cost'] + additional_cost
    return h_values


def compute_heuristics_potential_field(my_map, goal, wall_costs):
        
    nx = len(my_map)
    ny = len(my_map[0])
    goal_cost = np.zeros((nx, ny))

    h_values = dict()

    # computing cost for each cell on map 
    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            if my_map[i][j] == False:
                # considering the map as a potential field as a function of euclidean distance from walls, and manhattan distance from goal
                goal_cost[i][j] += np.exp(0.1*(abs(i-goal[0])+abs(j-goal[1])))
                h_values[(i,j)] = wall_costs[i][j] + goal_cost[i][j]
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


def is_constrained(curr_loc, child_loc, next_time, constraint_table):
    # function for handling all constraints
    blocked_loc = []
    constrained = None

    vertex_key = (child_loc, next_time)
    edge_key = (curr_loc, next_time)

    # handling vertex constraint
    if vertex_key in constraint_table:
        constraint = constraint_table[vertex_key]
        constrained = 'vertex'
        blocked_loc = constraint[constrained]
        return constrained, blocked_loc

    # handling edge constraint
    elif edge_key in constraint_table:
        constraint = constraint_table[edge_key]
        constrained = 'edge'
        blocked_loc = constraint[constrained]
        return constrained, blocked_loc
    return constrained, blocked_loc
    



def build_constraint_table(constraints, agent):
    table = dict()
    for i in constraints:
        if i['agent'] == agent:
            tuples_list = i['loc']
            constraint_key = ((tuples_list[-1], i['timestep']))
            constraint = {'edge': [tuples_list[0], tuples_list[-1]],
                          'vertex':[tuples_list[-1]]}
            table[constraint_key] = constraint
    return table

def constrain_path(path, constrained_agent, init_time=0, dt=1):
    constraints = []
    for j in range(len(path)):
        loc1 = get_location(path, j*dt)
        loc2 = get_location(path, j*dt+1)
        agent = constrained_agent
        constraint_dict = {'agent': agent,
                        'loc': [loc1,loc2],
                        'timestep': j*dt+init_time}
        constraints.append(constraint_dict)
    return constraints

def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, time_dependent=True, init_time=0):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    open_list = []
    closed_list = dict()
    constraint_table = build_constraint_table(constraints, agent)
    # setting up root node and adding it to open and closed list, to initiate the algorithm
    h_value = h_values[start_loc]
    root = {'loc': start_loc,
        'g_val': 0,
        'h_val': h_value,
        'timestep': init_time,
        'parent': None}
    push_node(open_list, root)
    closed_list[(root['loc'], 0)] = root

    dims = (len(my_map), len(my_map[0]))

    while len(open_list) > 0:
        curr = pop_node(open_list)
        curr_loc = curr['loc']
        curr_time = curr['timestep']
        child_time = curr['timestep'] + 1

        curr_cost = curr['g_val']+curr['h_val']

        constrained, blocked_loc = is_constrained(curr_loc, curr_loc, curr_time, constraint_table)

        if curr_loc == goal_loc and constrained == None: # if the current location is the goal location then return the path
            if curr_loc != blocked_loc:
                path = get_path(curr)
                return path

        for dir in range(5): # exploring all five moves that can be taken from the current location            
            child_loc = move(curr['loc'], dir)
            if -1<child_loc[0]<dims[0] and -1<child_loc[1]<dims[1]: # checking if the child location is within the map
                if my_map[child_loc[0]][child_loc[1]]: # if the child location is an obstacle then skip it
                    continue
                
                constrained, blocked_loc = is_constrained(curr_loc, child_loc, curr_time, constraint_table)

                # checking if direction is constrained, and what type of constraint, then creating child node accordingly
                if constrained == 'edge' and (child_loc == blocked_loc[0] or child_loc == blocked_loc[1]):
                    continue
                elif constrained == 'vertex' and child_loc == blocked_loc[0]:
                    continue
                else:
                    child = {'loc': child_loc,
                             'g_val': curr['g_val'] + 1,
                             'h_val': h_values[child_loc],
                             'timestep': child_time,
                             'parent': curr}
                    child_cost = child['g_val'] + child['h_val']
                child_key = (child['loc'], child['timestep']*time_dependent)
                
                if (child_key) in closed_list: 

                    existing_node = closed_list[child_key]
                    if compare_nodes(child, existing_node):
                        closed_list[child_key] = child
                        push_node(open_list, child)
                else:
                    a = {child['loc']}
                    closed_list[child_key] = child
                    push_node(open_list, child)
    return None  # Returns None if failed to find solutions
