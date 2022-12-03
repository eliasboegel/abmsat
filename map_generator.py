# script to generate a set number of map files each with different start and goal locations and different number of agents
import numpy as np, timeit

class CreateMaps(object):
    def __init__(self, file_name, agent_counts, input_string, sample_count):
        """Class to create a set number of map files with a given number of agents and a given map number"""
        self.file_name = file_name
        self.input_string = input_string
        
        for agent_num in agent_counts:
            self.file_addition_storage = []
            for sample_num in range(sample_count):
                # if 99 >= sample_num:
                #     self.file_name = file_name[0]
                # elif 199 >= sample_num >= 99:
                #     self.file_name = file_name[1]
                # elif 299 >= sample_num >= 199:
                #     self.file_name = file_name[2]
                # elif 399 >= sample_num >= 299:
                #     self.file_name = file_name[3]
                self.create_map(agent_num, sample_num+1500)

    def initialize_all_combos(self):
        """Initializes all possible start and goal locations for a given map """
        self.starts = []
        self.ends = []
        
        # generating all start and end locations
        for i in range(2):
            for j in range(9):
                start, end = str(j) + " " + str(i), str(j) + " " + str(20+i)
                self.starts.append(start)
                self.ends.append(end)
    
    def create_map(self, agent_num, map_num):
        """Creates a map file with a given number of agents and a given map number"""
        self.initialize_all_combos()

        file_addition = '\n' + str(agent_num)

        for i in range(agent_num):
            sample = np.random.randint(len(self.starts), size=2)
            start = self.starts.pop(sample[0])
            end = self.ends.pop(sample[1])
            file_addition += '\n' + start + " " + end

        # Checkin if the map already exist
        if file_addition in self.file_addition_storage:
            print('whoops, made a map which already exists')
            self.create_map(agent_num, map_num)
        else:
            # print('cool, map doesn\'t already exists')
            self.file_addition_storage.append(file_addition)
            
            output_string = self.input_string + file_addition

            # creating a map.txt file
            output_map = open(f'{self.file_name}{agent_num}_{map_num}.txt', 'w')
            output_map.write(output_string)
            output_map.close()


use_maps = ["map1","map2","map3"]

# Creating all the experiment map text files
for use_map in use_maps:
    skeletons = {
        "map1": "instances/map1_skeleton.txt",
        "map2": "instances/map2_skeleton.txt",
        "map3": "instances/map3_skeleton.txt"
    }
    file_name = "instances/experiment2_dis2/" + use_map + "_"
    skeleton_file = open(skeletons[use_map], 'r').read()
    # for prio and dis [2, 4, 6, 8, 10, 14, 18]
    # for cbs and cbscl [2, 4, 6, 8, 10, 12]
    print(f'\nCreating {use_map}\'s for {file_name}...')
    toc = timeit.default_timer()
    CreateMaps(file_name=file_name, agent_counts=[2, 4, 6, 8, 10, 14, 18], input_string=skeleton_file, sample_count=400)
    tic = timeit.default_timer()
    print(f'Finished creating all files in {round(tic-toc,5)} seconds')
