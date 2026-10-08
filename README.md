# Autonomous Warehouse Delivery Robot: Discrete Pathfinding Search Algorithms

An academic implementation and comparative evaluation of discrete graph search algorithms for an autonomous mobile robot navigating weighted warehouse floor grids. The project implements Uniform Cost Search (UCS) and A* Search with the Manhattan distance heuristic from scratch in Python, evaluating path optimality, state exploration, memory usage, and runtime across standard benchmark layouts.

## Project Overview

In an automated warehouse fulfillment center, a delivery robot transports goods from a starting dock to a designated destination while navigating obstacles and non-uniform surface friction. Because different terrain types incur different movement costs, the shortest route in terms of step count is not necessarily the cheapest route in terms of traversal cost. This project analyzes how uninformed and informed search algorithms handle this cost-distance tradeoff.

## Key Features

- Discrete 2D grid graph search implemented without third-party pathfinding libraries.
- Uninformed Uniform Cost Search (UCS) guaranteeing cost-optimal paths for positive edge weights.
- Informed A* Search with an admissible and consistent Manhattan distance heuristic ($L_1$ norm).
- Dynamic grid parsing and start/goal detection without hardcoded coordinates.
- Rigorous path verification validating coordinate continuity, obstacle avoidance, and cost calculations.
- Integrated ASCII trajectory rendering and Matplotlib visualization.
- Empirical benchmarking across solvable maps, cost-trap layouts, and provably unsolvable barrier maps.

## Grid and Cost Model

The warehouse environment is modeled as a 4-connected discrete grid with orthogonal movements: Up $(-1, 0)$, Down $(+1, 0)$, Left $(0, -1)$, and Right $(0, +1)$. Diagonal motion is prohibited.

| Symbol | Cell Meaning | Transition Cost |
| :--- | :--- | :--- |
| `S` | Start loading dock | 0 (initial state, not counted in traversal) |
| `G` | Goal delivery dock | 1 (entering docking cell) |
| `.` | Normal floor tile | 1 |
| `~` | Rough floor patch | 3 (elevated friction) |
| `#` | Structural wall barrier | Impassable (infinite cost) |

Step costs apply strictly upon entering each cell.

## Implemented Search Algorithms

### 1. Uniform Cost Search (UCS)
Uniform Cost Search is an uninformed graph search strategy. While standard Breadth-First Search assumes uniform edge costs and expands nodes by step depth, UCS systematically selects and expands the node with the lowest cumulative path cost $g(n)$ from the start state. Priority queue entries are ordered by $g(n)$, and the goal condition is evaluated when a state is popped from the queue, ensuring cost-optimality when all step costs are positive ($c \ge 1$).

### 2. A* Search (with Manhattan Distance)
A* Search is an informed graph search strategy that combines the exact historical path cost $g(n)$ with a forward-looking heuristic estimate $h(n)$ of the remaining distance to the goal:
$$f(n) = g(n) + h(n)$$

For 4-connected grid navigation, we use the Manhattan distance ($L_1$ norm):
$$h(n) = |r_n - r_G| + |c_n - c_G|$$

Because every grid transition costs at least 1 ($c_{\min} = 1$), Manhattan distance represents a true lower bound on obstacle-free travel. The heuristic is both admissible ($h(n) \le h^*(n)$) and consistent ($|h(n) - h(n')| \le 1 \le c(n, n')$), guaranteeing that A* returns an optimal path without reopening closed states.

## Benchmark Environments

1. **MAP_A (Standard Warehouse Layout, 5x8):** Contains structural wall blocks and localized rough terrain patches.
2. **MAP_B (Cost vs. Step Count Trap, 5x9):** Contains a direct rough-floor corridor (Row 0) and a longer clean-floor detour (Row 2).
3. **MAP_C (Unsolvable Barrier Map, 5x8):** Contains an unbroken vertical obstacle wall across Column 3 that completely isolates the goal from the start.

## Benchmark Results

The table below summarizes the verified performance metrics across all three maps:

| Map | Algorithm | Status | Path Length (Moves) | Path Cost | Nodes Expanded | Max Frontier Size | Runtime (ms) |
| :--- | :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **MAP_A** | Uniform Cost Search (UCS) | Success | 11 | 15 | 28 | 4 | 0.235 |
| **MAP_A** | A* Search | Success | 11 | 15 | 21 | 5 | 0.170 |
| **MAP_B** | Uniform Cost Search (UCS) | Success | 12 | 12 | 31 | 4 | 0.081 |
| **MAP_B** | A* Search | Success | 12 | 12 | 15 | 7 | 0.056 |
| **MAP_C** | Uniform Cost Search (UCS) | No path found | N/A | N/A | 13 | 3 | 0.041 |
| **MAP_C** | A* Search | No path found | N/A | N/A | 13 | 3 | 0.099 |

### Analysis of Key Findings

- **MAP_B Step Count vs. Cost Tradeoff:** Taking the direct rough corridor along Row 0 requires only 8 moves but crosses 7 rough tiles, resulting in a total cost of $7 \times 3 + 1 = 22$. Taking the clean floor detour around the wall via Row 2 requires 12 moves but only costs $11 \times 1 + 1 = 12$. Both UCS and A* correctly prioritize cumulative cost over move count and choose the 12-move route.
- **MAP_C Unsolvable Termination:** Because Column 3 is completely blocked by walls, only 13 nodes are reachable from the start dock. Both UCS and A* exhaustively explore all 13 reachable states, empty their priority queues, and terminate gracefully with the exact status `No path found`.
- **Search Efficiency Comparison:** On solvable maps (`MAP_A` and `MAP_B`), both algorithms return identical, cost-optimal paths. However, A* reduces node expansions by 25.0% on `MAP_A` (21 vs. 28) and by 51.6% on `MAP_B` (15 vs. 31) due to heuristic goal-directed pruning. Peak frontier size for A* is slightly larger on solvable maps (5 vs. 4 on `MAP_A`, 7 vs. 4 on `MAP_B`), reflecting the expected tradeoff of queuing promising forward neighbors earlier.
- **Algorithm Recommendation:** A* Search is recommended for this warehouse robot deployment on these benchmark environments because it delivers identical path cost-optimality while exploring significantly fewer states.

## Project Structure

```text
.
├── Warehouse_Delivery_Robot_Search_Algorithms.ipynb
├── README.md
└── .gitignore
```

The Jupyter Notebook is the single self-contained project artifact containing the complete Python source code, data structures, algorithms, benchmark execution runs, summary tables, and rendered visualization figures.

## Prerequisites and Installation

The project requires Python 3.9+ and standard data science libraries:

```bash
pip install pandas matplotlib numpy jupyter
```

Standard library modules utilized: `heapq`, `time`, `dataclasses`, `typing`.

## How to Run the Notebook

1. Clone the repository:
   ```bash
   git clone https://github.com/shaheenmohammedsh-ops/warehouse-delivery-robot-search-algorithms.git
   cd warehouse-delivery-robot-search-algorithms
   ```

2. Launch Jupyter Notebook or JupyterLab:
   ```bash
   jupyter notebook Warehouse_Delivery_Robot_Search_Algorithms.ipynb
   ```

3. Execute cells sequentially using **Cell -> Run All** or `Shift + Enter`.

4. **Single-Map Testing:** To test an individual map independently, change `GRID = MAP_B` or `GRID = MAP_C` in Section 3 and execute Section 9.1 (Configurable Single-Map Demonstration).

## Repository

- **GitHub:** [https://github.com/shaheenmohammedsh-ops/warehouse-delivery-robot-search-algorithms](https://github.com/shaheenmohammedsh-ops/warehouse-delivery-robot-search-algorithms)
