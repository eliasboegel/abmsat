# script to randomize starting and goal locations for set number of agents
import numpy as np

empty_map = open('01_skeleton_map.txt', 'r').read()
agent_count = 6
agent_string = str(agent_count)
locs = []

def loc_gen(existing_loc):
    a = np.random.randint(9)
    b = np.random.randint(2)

    c = np.random.randint(9)
    d = 21-np.random.randint(2)
    start = [a,b]
    goal = [c,d]
    if start in existing_loc or goal in existing_loc:
        start, goal = loc_gen(existing_loc)
    return start, goal
    

for i in range(agent_count):
    start, goal = loc_gen(locs)
    locs.append(start)
    locs.append(goal)
    agent_string += '\n' + str(start[0]) + ' ' + str(start[1]) + ' ' + str(goal[0]) + ' ' + str(goal[1])


print(agent_string)

final_string = empty_map + '\n' + agent_string

print(final_string)

file = open('0trial.txt', 'w')
file.write(final_string)
file.close()