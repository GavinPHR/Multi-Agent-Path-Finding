# Multi-Agent Path Finding

Anonymous Multi-Agent Path Finding (MAPF) with Conflict-Based Search (CBS) and Space-Time A* (STA*). This package does not assume that an agent can fit into a grid.

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
     assign:Callable = greedy_assign,
     max_iter:int = 200,
     low_level_max_iter = 500,
     debug:bool = False) -> np.ndarray
```
#### Parameters:
- **starts**: List[Tuple[int, int]] - A list of start coordinates.
- **goals**: List[Tuple[int, int]] - A list of goal coordinates.
- **assign**:Callable, *optional* - A function that assign each start coordinate to a goal coordinate. The default is to assign greedily each start coordinate to the closest goal coordinate.
- **max_iter**: int, *optional* - Max iterations of the high-level CBS algorithm. Default to `200`. 
- **low_level_max_iter**: int, *optional* - Max iterations of the low-level STA* algorithm. Default to `500`.
- **debug**: bool, *optional* - Prints some debug message. Default to `False`.

The size of **starts** and **goals** must be equal. 

The **assign** function returns a list of `Agent`, see `agent.py` for more details.

#### Return:
A `numpy.ndaarray` with shape `(N, L, 2)` with `N` being the number of agents (i.e. # of start coordinates) and `L` being the length of the path. 

To get the path of the first agent (i.e. the agent with the first start coordinates in `starts`), simply take the 0th-indexed element.

The individual path for each agent is padded to the same length `L`. 

## Theoretical Background

The high-level conflict-based search (CBS) is based on the paper [[Sharon et al. 2012]](https://www.aaai.org/ocs/index.php/AAAI/AAAI12/paper/viewFile/5062/5239). The low-level space-time A* (STA*) is like normal A* with an additional time dimension, [here](http://www0.cs.ucl.ac.uk/staff/D.Silver/web/Applications_files/coop-path-AIWisdom.pdf) is some information about it. 

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.


## License
[MIT](https://opensource.org/licenses/MIT)