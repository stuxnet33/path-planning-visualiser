# Path Planning Visualiser

Interactive visualisation of A* and RRT path planning algorithms built in Python. Draw mazes, set start and end points, and watch both algorithms navigate in real time.

---

## What it does

- Interactive grid — draw and erase walls with mouse clicks
- **A\*** — finds the guaranteed shortest path, visualised with blue exploration wave and yellow path
- **RRT** — probabilistic tree-based planner, visualised with purple tree and orange path
- Side by side comparison via algorithm switching — same maze, different algorithms
- Save and load mazes to JSON for reproducible testing
- Status bar shows path length, cells explored and iterations for each run

---

## Results

**A\* — optimal path, systematic exploration**
![A* result](astar.png)

**RRT — probabilistic tree, valid path**
![RRT result](rrt.png)

## Why both algorithms

A* and RRT solve the same problem in fundamentally different ways and each has strengths the other doesn't.

| | A* | RRT |
|---|---|---|
| Approach | Systematic grid search | Random tree sampling |
| Path quality | Optimal (shortest path) | Suboptimal (valid path) |
| Speed on grids | Fast | Slower, non-deterministic |
| Best use case | Discrete grid navigation | Continuous high-dimensional spaces |
| Handles tight mazes | Yes | Struggles — needs many iterations |

The visualiser makes these differences immediately obvious. A* finds the optimal path in milliseconds. RRT may need multiple attempts on the same maze but demonstrates probabilistic completeness — given enough iterations it will always find a path if one exists.

---

## Implementation details

**A\*** uses a priority queue (Python `heapq`) ordered by `f(n) = g(n) + h(n)` where `g` is actual cost from start and `h` is Euclidean distance heuristic. Allows 8-directional movement with diagonal cost `√2`.

**RRT** uses Bresenham's line algorithm for collision checking — the correct way to walk a line on a discrete grid without missing any cells. Goal biasing at 15% pulls the tree toward the goal. Automatic retry up to 3 attempts handles the probabilistic nature of the algorithm.

**Path reconstruction** is iterative in both algorithms — avoids Python's recursion limit on large trees and includes cycle detection for safety.

---

## Controls

| Key / Action | Function |
|---|---|
| Left click | Draw wall |
| Right click | Erase wall |
| S | Set start (green) |
| E | Set end (red) |
| Space | Run current algorithm |
| T | Toggle between A* and RRT |
| C | Clear search (keep walls) |
| R | Full reset |
| W | Save maze |
| L | Load maze |

---

## Stack

| Tool | Purpose |
|---|---|
| Python 3.14 | Core language |
| matplotlib | Grid rendering and animation |
| numpy | Grid data structures |

---

## Setup

```bash
git clone https://github.com/stuxnet33/path-planning-visualiser
cd path-planning-visualiser
python -m venv venv
venv\Scripts\activate
pip install matplotlib numpy
python main.py
```

---

## Planned

- RRT* — rewires tree for shorter paths
- Bidirectional RRT — grows trees from both start and goal simultaneously
- Maze generator — auto-generate random solvable mazes
- Side by side comparison view — run both algorithms simultaneously
- Export results as annotated PNG

---

*Part of a broader robotics and algorithms project series. See also: [arXiv Research Tracker](https://github.com/stuxnet33/arxiv-research-tracker) · [Self-Hosted VPN](https://github.com/stuxnet33/self-hosted-vpn)*
