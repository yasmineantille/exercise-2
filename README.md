# Exercise 2: Automated Planning

A partial implementation of a STRIPS-like planner in Python. The the planner solves a problem based on a domain, where both the *problem* and the *domain* are defined in [PDDL 1.2](https://planning.wiki/ref/pddl). The implemenation is based on [pyperplan](https://github.com/aibasel/pyperplan).

### Requirements
The project requires [Python](https://www.python.org/) >= 3.6.

### Task 2: Files of interest
- Complete the implementation of `task.py`
   - [`task.py`](https://github.com/HSG-WAS-SS22/exercise-2/blob/main/task.py): includes classes for creating Operators and STRIPS-like instances  ⟨Propositional arguments, Operators , Initial state , Goals⟩  
     - Implement the methods `applicable()` and `apply()` of the `Operator` class
     - Implement the methods `goal_reached()` and `get_successor_states()` of the `Task` class
- Complete the implementation of `search/a_star.py`
   - [`a_star.py`](https://github.com/HSG-WAS-SS22/exercise-2/blob/main/search/a_star.py): includes methods for searching based on the A* search algorithm
     - Complete the method `astar_search()` 
   - [`searchspace.py`](https://github.com/HSG-WAS-SS22/exercise-2/blob/main/search/searchspace.py): includes a `SearchNode` class for creating instance nodes that are visited during search 

### Task 3: Files of interest
- Examine the `task-3/t3-domain.py` and the `task-3/t3-problem.py`, and the behavior of the planner when solving the problem in the domain, so that you answer the questions provided to you in the exescise sheet. 
  - [`t3-domain.py`](https://github.com/HSG-WAS-SS22/exercise-2/blob/main/task-3/t3-domain.pddl): the domain
  - [`t3-problem.py`](https://github.com/HSG-WAS-SS22/exercise-2/blob/main/task-3/t3-problem.pddl): the problem

### Project structure
```bash
├── benchmarks # benchmark examples with domain and problem use cases, which you can use to see the planner behavior
├── examples # simple example of a "block" domain and problem, which you can use to see the planner behavior
│   ├── blocks-world-domain.pddl 
│   └── blocks-world-problem.pddl
├── search # includes implementations of different search algorithms (i.e. astar,wastar,gbf,bfs,ehs,ids,sat)
│   ├── a_star.py # (altered in Task 2) template of A* search algorithm 
│   ├── breadth_first_search.py
│   ├── ...
│   └── searchspace.py # (used in Task 2) includes a class for creating instance nodes that are visited during search 
├── task.py # (altered in for Task 2)  includes classes for creating Operators and STRIPS-like instances  ⟨Propositional arguments, Operators , Initial state , Goals⟩  
├── plan.py # (used in Task 2) the main script for solving a planning problem 
├── pddl # a PDDL parser 
├── heuristics # includes implementations of different heuristic methods (i.e. blind,landmark,lmcut,hadd,hff,hmax,hsa)
├── grounding.py # grounding a schematic PDDL task to a STRIPS planning task
├── planner.py # a STRIPS-like planner
├── ...
└── README.md # this README.md
```


### How to run the project 
You can run the script `plan.py` with Python 3 for solving a problem defined in PDDL.
You can specify the following arguments:
- positional arguments:
  - `domain`: the file path of the domain defined in PDDL
  - `problem`: the file path of the problem defined in PDDL

- optional arguments:
  - `-s {astar,wastar,gbf,bfs,ehs,ids,sat}`: the search algorithm from A*, weighted A*, greedy best first, breadth first, enforced hillclimbing, iterative deepening, sat solve (default: bfs)

```
python3 plan.py [-s {astar,wastar,gbf,bfs,ehs,ids,sat}] [domain] problem
```
For example, to run the planner with the breadth-first algorithm:
```
python3 plan.py -s bfs examples/blocks-world-domain.pddl examples/blocks-world-problem.pddl
```
Or to run the planner with the A* algorithm that you implemented in Task 2:
```
python3 plan.py -s astar t1-your-domain.pddl t1-your-problem.pddl
```
