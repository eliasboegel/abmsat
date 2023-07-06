# ABMSAT, main and dev branches are the latest versions as of 3rd Dec 2022.
Course practical for AE4422-20 Agent-based Modelling and Simulation for Air Transportation.

This git repo contains implementations of several path planners that are used to solver an arbitrary multiagent path finding problem. The solutions include:

- a prioritize A* path planner,
- a CBS and CBS cycle limited path planner,
- and a distributed path planner.

To run the code, clone this repository onto your local machine. Open command prompt at the directory where you cloned the repo, and run the command:

`python run_experiments.py --solver solver_of_choice --instance instance_of_choice --hvals heuristic_of_choice(optional)`

For solver_of_choice you can enter any of the following solvers: 

- `Independent` (picks the independent solver, has no collision avoidance)
- `Prioritized` (picks the Prioritized A* solver)
- `CBS`         (picks the conflict based search solver) 
- `CBSCL`       (picks the conflict based search with cycle limiting solver) 
- `Distributed` (picks the distributed planning solver)

For instance_of_choice you can enter the path to the text file containing the map of your choice, a large collection of maps are presente already in the \instances\ directory. An example of instance_of_choice is:

- `instances\map1.txt`

For heuristics_of_choice, note this is an optional argument, but you can pick to enter either:
- `old` (picks the old heuristics map)
- `goals` (picks the heursitics map with extra costs on goal cells)

Example:
`python run_experiments.py --solver Distributed --instance instances\map1.txt`

## Below is a gif demonstrating the CBS cycle limited solver solving a sliding puzzle, with each puzzle piece being an agent.

![sliding_puzzle_1](https://user-images.githubusercontent.com/65394178/201221764-6fe19081-493b-41df-993d-4440e71d35ca.gif)
