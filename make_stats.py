"""
Python script to run monte carlo simulations and save all the generated data into /stats/.

"""

#!/usr/bin/python
import argparse
import glob
import timeit
from pathlib import Path
from cbs import CBSSolver
from cbscl import CBSCLSolver
from prioritized import PrioritizedPlanningSolver
from distributed import DistributedPlanningSolver
from single_agent_planner import get_trip_length, get_trip_duration, a_star, compute_heuristics, compute_heuristics_goals
import numpy as np
import matplotlib.pyplot as plt
from visualize import Animation

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
    parser.add_argument('--hvals', type=str, default=None,
                        help='The name of the heuristic map')

    args = parser.parse_args()


    # all_solvers = ['CBS', 'CBSCL', 'Prioritized', 'Distributed']
    # all_solvers = ['CBSCL', 'Prioritized', 'Distributed']
    # all_solvers = ['Prioritized']
    all_solvers = ['Distributed']
    heuristics_list = ['goals']
    experiment_time = timeit.default_timer()

    for heuristics_name in heuristics_list:
        print('\n')
        toob = timeit.default_timer()
        for solver_string in all_solvers:
            t = 0
            exp_samples = 1

            exp_sample_list = []
            last_exp_id = ''
            experiments_dict = {}

            print(f'\n******Running {solver_string} solver with {heuristics_name} heuristics******\n')

            result_file = open(f"stats/{solver_string}-{heuristics_name}_results.csv", "w")
            result_file.write(f"map_id,trip durations,trip lengths,runtime[s],duration/len(a*)\n")

            for file in sorted(glob.glob(args.instance)):
                t+=1
                tong = timeit.default_timer()
                # displaying progress
                if t%50 == 0:
                    print(f"Time passed: {round(tong - toc,5)} seconds (solver time: {round(tong-toob,8)} s)\n   {t}/{len(glob.glob(args.instance))} maps done")

                my_map, starts, goals = import_mapf_instance(file)

                # h_vals = []
                # if heuristics_name == 'old':
                #     for goal in goals:
                #         h_vals.append(compute_heuristics(my_map, goal))
                # elif heuristics_name == 'goals':
                #     for goal in goals:
                #         h_vals.append(compute_heuristics_goals(my_map, goal, goals))

                # print("***Import an instance***")
                if solver_string == "CBS":
                    solver = CBSSolver(my_map, starts, goals, heuristics_name)
                elif solver_string == "CBSCL":
                    solver = CBSCLSolver(my_map, starts, goals, heuristics_name)
                elif solver_string == "Prioritized":
                    solver = PrioritizedPlanningSolver(my_map, starts, goals, heuristics_name)
                elif solver_string == "Distributed":  # Wrapper of distributed planning solver class
                    solver = DistributedPlanningSolver(my_map, starts, goals, heuristics_name) #!!!TODO: add your own distributed planning implementation here.
                else: 
                    raise RuntimeError("Unknown solver!")

                durations_normalized = 0

                #spliting the file string to obtain the map id
                map_id = file.split('\\')[-1].replace('.','_').split('_')[0:3]
                existing_keys = experiments_dict.keys()
                experiment_id = solver_string + '_' + map_id[0] + '_' + map_id[1]
                
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

                    a_star_lengths = np.zeros(len(paths))
                    path_durations = np.zeros(len(paths))

                    paths_lengths = np.zeros(len(paths))
                    for i in range(len(paths)):
                        # getting a_star paths for each agent, so we can find the normalized duration of each agent's path
                        a_star_lengths[i] = get_trip_length(a_star(my_map, starts[i], goals[i], h_vals[i], i, []))

                        # getting the length of each agent's path
                        path = paths[i]
                        paths_lengths[i] = len(path)

                        # recording each agent's path duration
                        path_durations[i] = get_trip_duration([path])
                    
                    max_paths_lengths = int(np.max(paths_lengths))
                    for i in range(max_paths_lengths):
                        for j in range(len(paths)-1):
                            for k in range(j+1, len(paths)):
                                cell1 = paths[j][i] if i < len(paths[j]) else paths[j][-1]
                                cell2 = paths[k][i] if i < len(paths[k]) else paths[k][-1]
                                if cell1 == cell2:
                                    raise Exception("agents collided")

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

                    durations_normalized_entry = str(durations_normalized).replace('[','').replace(']','').replace(', ','-')
                    result_file.write(f"{file},{trip_duration},{trip_length},{solver_time},{durations_normalized_entry}\n")

                    # animation = Animation(my_map, starts, goals, paths)
                    # animation.show()
                except:
                    # print('error!!')
                    result_file.write(f"{file},{999999},{999999},{999999},{999999}\n")

            tic = timeit.default_timer()
            experiment_time = round(tic - toc,8)
            print(f'Finished experiments for {solver_string} with {heuristics_name} heuristics!!\nTime elapsed: {experiment_time} seconds')

            # recording all the statistics for each experiment
            stats_file = open(f"stats/{solver_string}-{heuristics_name}_stats.csv", "w")
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
    final_toc = timeit.default_timer()
    print(f'\n\n******Finished all experiments!!****** \nTime elapsed: {round(final_toc - toc,8)} seconds')
