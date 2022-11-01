# script to randomize starting and goal locations for set number of agents
import numpy as np
class CreateMaps(object):
    def __init__(self, agent_counts, input_string, sample_count):
        self.input_string = input_string
        
        for agent_num in agent_counts:
            for sample_num in range(sample_count):
                self.create_map(agent_num, sample_num)

    def initialize_all_combos(self):
        self.starts = []
        self.ends = []
        
        # generating all start and end locations
        for i in range(2):
            for j in range(9):
                start, end = str(j) + " " + str(i), str(j) + " " + str(20+i)
                self.starts.append(start)
                self.ends.append(end)
    
    def create_map(self, agent_num, map_num):
        self.initialize_all_combos()

        file_addition = '\n' + str(agent_num)

        for i in range(agent_num):
            sample = np.random.randint(len(self.starts), size=2)
            start = self.starts.pop(sample[0])
            end = self.ends.pop(sample[1])
            file_addition += '\n' + start + " " + end

        print(file_addition)
        
        output_string = self.input_string + file_addition

        output_map = open(f'map1_{agent_num}_{map_num}.txt', 'w')
        output_map.write(output_string)
        output_map.close()


skeleton_file = open('01_skeleton_map.txt', 'r').read()
CreateMaps(agent_counts=[6,8,10], input_string=skeleton_file, sample_count=8)