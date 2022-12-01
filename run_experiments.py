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
from single_agent_planner import get_trip_length, get_trip_duration, a_star, compute_heuristics, compute_heuristics_goals, compute_heuristics_potential_field
import numpy as np
import matplotlib.pyplot as plt

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
    parser.add_argument('--hvals', type=str, default=None,
                        help='The name of the heuristic map')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,CBSCL,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()

    t = 0
    exp_samples = 1
    exp_sample_list = []
    last_exp_id = ''
    experiments_dict = {}

    print(f'\n******Running {args.solver} solver******\n')
    if args.batch:
        result_file = open(f"stats/{args.solver}_results.csv", "w")
        result_file.write(f"map_id,trip durations,trip lengths,runtime[s],duration/len(a*)\n")

    for file in sorted(glob.glob(args.instance)):
        # print(file, '\n\n\n')
        t+=1
        tong = timeit.default_timer()
        # displaying progress
        # if t%30 == 0:
        print(f"Time passed: {round(tong - toc,5)} seconds")

        my_map, starts, goals = import_mapf_instance(file)

        heuristics_name = args.hvals

        # h_vals = []
        # if heuristics_name == 'old':
        #     for goal in goals:
        #         h_vals.append(compute_heuristics(my_map, goal))
        # elif heuristics_name == 'goals':
        #     for goal in goals:
        #         h_vals.append(compute_heuristics_goals(my_map, goal, goals))
        # else:
        #     for goal in goals:
        #         h_vals.append(compute_heuristics(my_map, goal))

        # print("***Import an instance***")
        if args.solver == "CBS":
            solver = CBSSolver(my_map, starts, goals, args.hvals)
        elif args.solver == "CBSCL":
            solver = CBSCLSolver(my_map, starts, goals, args.hvals)
        elif args.solver == "Independent":
            solver = IndependentSolver(my_map, starts, goals, args.hvals)
        elif args.solver == "Prioritized":
            solver = PrioritizedPlanningSolver(my_map, starts, goals, args.hvals)
        elif args.solver == "Distributed":  # Wrapper of distributed planning solver class
            solver = DistributedPlanningSolver(my_map, starts, goals, args.hvals) #!!!TODO: add your own distributed planning implementation here.
        else: 
            raise RuntimeError("Unknown solver!")

        if args.batch:
            durations_normalized = 0

            #spliting the file string to obtain the map id
            map_id = file.split('\\')[-1].replace('.','_').split('_')[0:3]
            existing_keys = experiments_dict.keys()
            experiment_id = args.solver + '_' + map_id[0] + '_' + map_id[1]
            # print(experiment_id)
            if experiment_id == last_exp_id:
                exp_samples += 1
            else:
                exp_sample_list.append(exp_samples)
                exp_samples = 1
            last_exp_id = experiment_id
            # lazy error handling try/except statements :(
            try:
                solve_start = timeit.default_timer()
                paths = solver.find_solution()
                solve_end = timeit.default_timer()

                # Calculating standard deviation in arrival times
                if "Distributed" in args.solver:
                    heuristics = []
                    for goal in goals:
                        heuristics.append(compute_heuristics(my_map, goal))
                else:
                    heuristics = solver.heuristics

                a_star_lengths = np.zeros(len(paths))
                path_durations = np.zeros(len(paths))
                for i in range(len(paths)):
                    a_star_lengths[i] = get_trip_length(a_star(my_map, starts[i], goals[i], heuristics[i], i, []))
                
                # a_star_lengths = np.ones(len(paths))*np.mean(a_star_lengths)

                for i in range(len(paths)):
                    path = paths[i]
                    path_durations[i] = get_trip_duration([path])

                durations_normalized = list(path_durations/a_star_lengths)

                solver_time = round(solve_end - solve_start, 8)
                trip_duration = get_trip_duration(paths)
                trip_length = get_trip_length(paths)

                
                if experiment_id not in existing_keys:
                    experiments_dict[experiment_id] = [[trip_duration] , [trip_length], [solver_time], durations_normalized]
                elif experiment_id in existing_keys:
                    experiments_dict[experiment_id][0] += [trip_duration]
                    experiments_dict[experiment_id][1] += [trip_length]
                    experiments_dict[experiment_id][2] += [solver_time]
                    experiments_dict[experiment_id][3] += durations_normalized

                result_file.write(f"{file},{trip_duration},{trip_length},{solver_time},{durations_normalized}\n")
            except:
                result_file.write(f"{file},{999999},{999999},{999999},{999999}\n")
            # print(f'durations normalized: {durations_normalized}')
                 
        elif not args.batch:
            solve_start = timeit.default_timer()
            paths = solver.find_solution()
            solve_end = timeit.default_timer()

            solver_time = round(solve_end - solve_start, 8)
            trip_duration = get_trip_duration(paths)
            trip_length = get_trip_length(paths)

            print(f'Solver returned solution: {paths}\nTotal trip duration for all agents: {trip_duration}\nTotal trip length for all agents: {trip_length}\nSolver time: {solver_time}\n')
            
            print_mapf_instance(my_map, starts, goals)

            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.animate_continuously()
            # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
            animation.show()


    tic = timeit.default_timer()
    total_time = round(tic - toc,8)
    print(f'\n\n******Finished all experiments!!****** \nTotal Time elapsed: {total_time} seconds')
    # print(f'loop ran {t} times')
    if args.batch:
        result_file.close()


        # recording all the statistics for each experiment
        stats_file = open(f"stats/{args.solver}_stats.csv", "w")
        stats_file.write(f"map_id,avg trip duration,avg trip length,successful samples,total samples,runtime [s],sttdev arrival times\n")
        all_keys = list(experiments_dict.keys())
        for key in all_keys:

            # key = all_keys[i]
            samples_taken = exp_sample_list[-1]

            trip_durations = experiments_dict[key][0]
            trip_lengths = experiments_dict[key][1]
            solver_times = experiments_dict[key][2]
            durations_normalized = experiments_dict[key][3]
            # print(durations_normalized)

            avg_durations = sum(trip_durations)/len(trip_durations)
            avg_lengths = sum(trip_lengths)/len(trip_lengths)
            successful_samples = len(trip_durations)
            avg_solver_times = sum(solver_times)/len(solver_times)
            sttd_durations = np.std(np.array(durations_normalized))
            
            stats_file.write(f"{key},{avg_durations},{avg_lengths},{successful_samples},{samples_taken},{avg_solver_times},{sttd_durations}\n")

            # plt.hist(durations_normalized, 100, density=True, facecolor='g', alpha=0.75)
            # plt.show()
    



