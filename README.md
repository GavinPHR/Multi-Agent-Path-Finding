# Multi-Agent Path Finding ![](https://img.shields.io/pypi/v/cbs-mapf) ![](https://img.shields.io/badge/python-%3E%3D3.5-blue) ![](https://img.shields.io/github/license/GavinPHR/Multi-Agent-Path-Finding) 

Anonymous Multi-Agent Path Finding (MAPF) with Conflict-Based Search (CBS) and Space-Time A* (STA*). I strongly recommend you to also check out my [Space-Time A*](https://github.com/GavinPHR/Space-Time-AStar) repository for a complete picture of how this package works.

 - [Visualization](#visualization)
 - [Installation](#installation)
 - [Usage](#usage)
 - [Theoretical Background](#theoretical-background)
 - [Contributing](#contributing)
 - [License](#license)

## Visualization 

4 agents at the 4 corners of the map trying to align themselves in the middle:

<p align="center">
  <img width="500" src="https://raw.githubusercontent.com/GavinPHR/Multi-Agent-Path-Finding/master/fig/visualization1.gif">
</p>

16 agents in a circlular layout trying to form a grid layout:

<p align="center">
  <img width="600" src="https://raw.githubusercontent.com/GavinPHR/Multi-Agent-Path-Finding/master/fig/visualization2.gif">
</p>

The above visualizations are generated using `visualizer.py` and scenario yaml files in the [visualization folder](https://github.com/GavinPHR/Multi-Agent-Path-Finding/tree/master/visualization). Once the package (and other [requirements](https://github.com/GavinPHR/Multi-Agent-Path-Finding/blob/master/visualization/requirements.txt)) is installed, you can generate them like so:

```bash
python3 visualizer.py scenario1.yaml
python3 visualizer.py scenario2.yaml
```

## Installation

The package is named `cbs-mapf` and listed on [PyPI](https://pypi.org/project/cbs-mapf/). You can use the pip to install:

```bash
pip3 install cbs-mapf
```

This will also install its sister package `space-time-astar`, also on [GitHub](https://github.com/GavinPHR/Space-Time-AStar) and [PyPI](https://pypi.org/project/space-time-astar/).

## Usage

### Import Planner

```python
from cbs_mapf.planner import Planner
```

### Constructor Parameters
```python
Planner(grid_size: int, robot_radius: int, static_obstacles: List[Tuple[int, int]])
```
- **grid_size**: int - each grid will be square with side length **grid_size**. 
- **robot_radius**: int - agents are assumed to be circles with radius being **robot_radius**.
- **static_obstacles**: List[Tuple[int, int]] - A list of coordinates specifying the static obstacles in the map.

It is important that **grid_size** is not too small and **static_obstacles** does not contain too many coordinates, otherwise the performance will severely deteriorate. What is 'too many' you might ask? It depends on your requirements.

### Find Path
Use `Planner`'s `plan()` method:
```
plan(starts: List[Tuple[int, int]],
     goals: List[Tuple[int, int]],
     assign:Callable = min_cost,
     max_iter:int = 200,
     low_level_max_iter:int = 100,
     max_process:int = 10,
     debug:bool = False) -> np.ndarray
```
#### Parameters:
- **starts**: List[Tuple[int, int]] - A list of start coordinates.
- **goals**: List[Tuple[int, int]] - A list of goal coordinates.
- **assign**: Callable, *optional* - A function that assign each start coordinate to a goal coordinate. The default assignment yields a minimum cost with repesct to straight line euclidean distances, see [Hungarian Algorithm](https://en.wikipedia.org/wiki/Hungarian_algorithm).
- **max_iter**: int, *optional* - Max iterations of the high-level CBS algorithm. Default to `200`. 
- **low_level_max_iter**: int, *optional* - Max iterations of the low-level STA* algorithm. Default to `100`.
- **max_process**: int, *optional* - Maximum number of processes to spawn. Default to `10`.
- **debug**: bool, *optional* - Prints some debug message. Default to `False`.

The size of **starts** and **goals** must be equal. 

The **assign** function returns a list of `Agent`, see `agent.py` for more details.

#### Return:
A `numpy.ndaarray` with shape `(N, L, 2)` with `N` being the number of agents (i.e. # of start coordinates) and `L` being the length of the path. 

To get the path of the first agent (i.e. the agent with the first start coordinates in `starts`), simply take the 0th-indexed element.

The individual path for each agent is padded to the same length `L`. 

## Theoretical Background

The high-level conflict-based search (CBS) is based on the paper [[Sharon et al. 2012]](https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239). The low-level space-time A\* (STA\*) is like normal A\* with an additional time dimension, see the [GitHub page](https://github.com/GavinPHR/Space-Time-AStar) for more information. 

### Constraint Tree

The core of the algorithm is the maintenance of a constraint tree (a binary min-heap in my implementation). The nodes in the constraint tree have 3 component:

- constraints - detailing what each agent should avoid in space-time
- solution - path for each agent
- cost - the sum of the cost of individual paths

The low-level STA* planner can take the constraints for an agent and calculate a collison-free path for that agent.

### Pseudocode 

Taken from [[Sharon et al. 2012]](https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239) with minor modification.

```
node = Find paths for individual agents with no constraints.
Add node to the constraint tree.

while constraint tree is not empty:
  best = node with the lowest cost in the constraint tree

  Validate the solution in best until a conflict occurs.
  if there is no conflict:
    return best

  conflict = Find the first 2 agents with conflicting paths.

  new_node1 = node where the 1st agent avoid the 2nd agent
  Add new_node1 to the constraint tree.

  new_node2 = node where the 2nd agent avoid the 1st agent
  Add new_node2 to the constraint tree.
```


## Contributing
Suggestions and pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.


## License
[MIT](https://opensource.org/licenses/MIT)
