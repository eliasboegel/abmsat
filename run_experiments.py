"""
Main file to run experiments and show animation.

Note: To make the animation work in Spyder you should set graphics backend to 'Automatic' (Preferences > Graphics > Graphics Backend).
"""

#!/usr/bin/python
import csv
import argparse
import glob
import timeit
from pathlib import Path
from cbs import CBSSolver
from cbscl import CBSCLSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from distributed import DistributedPlanningSolver # Placeholder for Distributed Planning
from visualize import Animation
from single_agent_planner import get_sum_of_cost, get_trip_length

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    """
    Prints start location and goal location of all agents, using @ for an obstacle, . for a open cell, and 
    a number for the start location of each agent.
    
    Example:
        @ @ @ @ @ @ @ 
        @ 0 1 . . . @ 
        @ @ @ . @ @ @ 
        @ @ @ @ @ @ @ 
    """
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    """
    See docstring print_mapf_instance function above.
    """
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    """
    Imports mapf instance from instances folder. Expects input as a .txt file in the following format:
        Line1: #rows #columns (number of rows and columns)
        Line2-X: Grid of @ and . symbols with format #rows * #columns. The @ indicates an obstacle, whereas . indicates free cell.
        Line X: #agents (number of agents)
        Line X+1: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 1)
        Line X+2: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 2)
        Line X+n: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent n)
        
    Example:
        4 7             # grid with 4 rows and 7 columns
        @ @ @ @ @ @ @   # example row with obstacle in every column
        @ . . . . . @   # example row with 5 free cells in the middle
        @ @ @ . @ @ @
        @ @ @ @ @ @ @
        2               # 2 agents in this experiment
        1 1 1 5         # agent 1 starts at (1,1) and has (1,5) as goal
        1 2 1 4         # agent 2 starts at (1,2) and has (1,4) as goal
    """
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':

    toc = timeit.default_timer()
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,CBSCL,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()
    result_file = open("results.csv", "w", buffering=1)

    t = 0
    experiments_dict = {}

    print(f'\n******Running {args.solver} solver******\n')
    result_file.write(f"map_id,trip durations,trip lengths\n")
    for file in sorted(glob.glob(args.instance)):
        t+=1
        tong = timeit.default_timer()
        if t%100 == 0:
            print(f"Time passed: {round(tong - toc,5)} seconds")


        my_map, starts, goals = import_mapf_instance(file)

        if not args.batch:
            print_mapf_instance(my_map, starts, goals)

        # lazy error handling try/except statements :(
        try:
        # print("***Import an instance***")
            if args.solver == "CBS":
                solver = CBSSolver(my_map, starts, goals)
            elif args.solver == "CBSCL":
                solver = CBSCLSolver(my_map, starts, goals)
            elif args.solver == "Independent":
                solver = IndependentSolver(my_map, starts, goals)
            elif args.solver == "Prioritized":
                solver = PrioritizedPlanningSolver(my_map, starts, goals)
            elif args.solver == "Distributed":  # Wrapper of distributed planning solver class
                solver = DistributedPlanningSolver(my_map, starts, goals) #!!!TODO: add your own distributed planning implementation here.
            else: 
                raise RuntimeError("Unknown solver!")
            paths = solver.find_solution()

            trip_duration = get_sum_of_cost(paths)
            trip_length = get_trip_length(paths)
            
            #spliting the file string to obtain the map id
            map_id = file.split('\\')[-1].replace('.','_').split('_')[0:3]
            existing_keys = experiments_dict.keys()
            experiment_id = args.solver + '_' + map_id[0] + '_' + map_id[1]

            if experiment_id not in existing_keys:
                experiments_dict[experiment_id] = [[trip_duration] , [trip_length]]
            elif experiment_id in existing_keys:
                experiments_dict[experiment_id][0] += [trip_duration]
                experiments_dict[experiment_id][1] += [trip_length]

            # print(f'\n\nahhhh {experiments_dict}')
            
            # print(f"Sum of lengths: {trip_length}")
            result_file.write(f"{file},{trip_duration},{trip_length}\n")
        except:
            result_file.write(f"{file},{99999}\n")



        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.animate_continuously()
            # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
            animation.show()
    result_file.close()

    stats_file = open("stats.csv", "a")

    all_keys = experiments_dict.keys()
    for key in all_keys:
        trip_durations = experiments_dict[key][0]
        trip_lengths = experiments_dict[key][1]
        
        avg_durations = sum(trip_durations)/len(trip_durations)
        avg_lengths = sum(trip_lengths)/len(trip_lengths)

        successful_samples = len(trip_durations)
        # print(f'{key},{trip_durations},{trip_lengths}')
        
        stats_file.write(f"{key},{avg_durations},{avg_lengths},{successful_samples}\n")



    tic = timeit.default_timer()
    print(f'\n\n******Finished all experiments!!****** \nTotal Time elapsed: {round(tic - toc,5)} seconds')
