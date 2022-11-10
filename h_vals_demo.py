# import os

# for filename in os.listdir(f'{os.getcwd()}/test/'):
#     with open(os.path.join(f'{os.getcwd()}/test/', filename), 'r') as f: # open in readonly mode
#         print(f.read())
#       # do your stuff

import numpy as np
from matplotlib import pyplot, cm
from mpl_toolkits.mplot3d import Axes3D
from run_experiments import import_mapf_instance, print_mapf_instance
from single_agent_planner import compute_heuristics, compute_heuristics_goals, compute_heuristics_potential_field


def plot2D(x, y, p, title):
  fig = pyplot.figure(figsize=(11, 7), dpi=100)
  fig.suptitle(title, fontsize=20)
  ax = fig.gca(projection='3d')
  X, Y = np.meshgrid(x, y)
  ax.plot_surface(X, Y, p[:], rstride=1, cstride=1, cmap=cm.viridis,
          linewidth=0, antialiased=False)
  ax.view_init(30, 45)
  ax.set_xlabel('$x$')
  ax.set_ylabel('$y$')
  ax.set_box_aspect((9,22,9))
  

def find_wall_costs(cell, my_map):
    cost = 0
    x, y = cell[0], cell[1]
    for i in range(len(my_map)):
        for j in range(len(my_map[0])):
            if my_map[i][j]:
                cost += 3*np.exp(-2*np.sqrt((x-i)**2 + (y-j)**2))
    return cost


def compute_heuristics_potential_field(my_map, goal, wall_costs):
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



my_map, starts, goals = import_mapf_instance("instances/map1.txt")
print_mapf_instance(my_map, starts, goals)

nx = len(my_map)
ny = len(my_map[0])
wall_cost = np.zeros((nx, ny))
for i in range(len(my_map)):
    for j in range(len(my_map[0])):
        if my_map[i][j] == False:
            wall_cost[i][j] = find_wall_costs((i,j), my_map)

# Parameters
xx, yy = 1, 1
nx = len(my_map)*xx
ny = len(my_map[0])*yy
nt  = 1000
xmin = 0
xmax = len(my_map)
ymin = 0
ymax = len(my_map[0])

# Initialization
wall_costs = np.zeros((nx, ny))
for i in range(len(my_map)):
    for j in range(len(my_map[0])):
        if my_map[i][j] == False:
            wall_cost[i][j] = find_wall_costs((i,j), my_map)
            
start, goal = starts[0], goals[0]

x  = np.linspace(xmin, xmax, nx)
y  = np.linspace(ymin, ymax, ny)


h_vals = compute_heuristics(my_map, goal)
h_vals_own = compute_heuristics_potential_field(my_map, goal, wall_costs)
h_vals_goals = compute_heuristics_goals(my_map, goal, goals)

h_cost = np.zeros((nx, ny))
h_cost_own = np.zeros((nx, ny))
h_cost_goals = np.zeros((nx, ny))

for i in range(len(my_map)):
    for j in range(len(my_map[0])):
        if my_map[i][j] == False:
            key = (i,j)
            print(f'key: {key}')
            h_cost[i][j] = h_vals[key]
            h_cost_own[i][j] = h_vals_own[key]
            h_cost_goals[i][j] = h_vals_goals[key]
          
# for i in range(len(my_map)):
#     for j in range(len(my_map[0])):
#         if my_map[i][j] == False:
#           goal_cost[i][j] += np.exp(0.15*(abs(i-goal[0])+abs(j-goal[1])))

plot2D(x, y, np.transpose(h_cost), 'Old heursitics value map')
# print(h_cost[:])
plot2D(x, y, np.transpose(h_cost_own), 'New heursitics value map')
plot2D(x, y, np.transpose(h_cost_goals), 'Heursitics value map with goals')
pyplot.show()