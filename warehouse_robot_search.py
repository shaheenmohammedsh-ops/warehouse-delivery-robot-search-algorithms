"""
Autonomous Warehouse Delivery Robot: Discrete Pathfinding Search Algorithms
=============================================================================

Course: Intelligent Systems / Artificial Intelligence
Topic: Discrete Graph Search Algorithms (Uniform Cost Search vs. A* Search)

How to Run:
-----------
To execute the complete benchmark suite and active GRID demonstration, run:
    python warehouse_robot_search.py

How to Change the Active Map (GRID):
------------------------------------
Modify the `GRID` variable below (Line 55) to any defined map:
    GRID = MAP_A   # Standard warehouse layout (default)
    GRID = MAP_B   # Cost vs. step-count tradeoff map
    GRID = MAP_C   # Unsolvable barrier map
"""

import heapq
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple


# ==============================================================================
# BENCHMARK MAP CONFIGURATIONS
# ==============================================================================

MAP_A: List[str] = [
    "S..#....",
    "##.#.##.",
    "....~~..",
    ".####.#.",
    "...~..#G",
]

MAP_B: List[str] = [
    "S~~~~~~~G",
    ".#######.",
    ".........",
    ".#.#.#.#.",
    ".........",
]

MAP_C: List[str] = [
    "S..#....",
    ".#.#.##.",
    "...#..#.",
    ".###.##.",
    "....#..G",
]

# Configurable active map variable (default: MAP_A)
GRID: List[str] = MAP_A

# Semantic transition cost dictionary
COST_TABLE: Dict[str, int] = {
    '.': 1,   # Normal floor
    '~': 3,   # Rough floor
    'G': 1,   # Goal docking cell
    'S': 0,   # Start cell (not counted in traversal cost)
}

WALL_CHAR: str = '#'
VALID_CELL_CHARS: Set[str] = {'.', '~', 'G', 'S', WALL_CHAR}

# 4-connected movement action offsets: (delta_row, delta_col)
DIRECTIONS: List[Tuple[int, int]] = [
    (-1, 0),  # Up
    (1, 0),   # Down
    (0, -1),  # Left
    (0, 1),   # Right
]

Coordinate = Tuple[int, int]


# ==============================================================================
# DATA STRUCTURES & UTILITIES
# ==============================================================================

@dataclass(frozen=True)
class GridInfo:
    """Encapsulates a validated warehouse grid environment."""
    grid: List[str]
    rows: int
    cols: int
    start: Coordinate
    goal: Coordinate


@dataclass
class SearchResult:
    """Standardized container for search execution metrics."""
    algorithm: str
    map_name: str
    found: bool
    path: List[Coordinate] = field(default_factory=list)
    length: int = 0
    cost: float = float('inf')
    nodes_expanded: int = 0
    max_frontier_size: int = 0
    execution_time_ms: float = 0.0

    @property
    def status(self) -> str:
        return 'Success' if self.found else 'No path found'


def parse_and_validate_grid(grid: List[str]) -> GridInfo:
    """
    Validates grid structure and dynamically locates 'S' and 'G'.
    Enforces non-emptiness, rectangularity, permitted characters, and unique S/G cells.
    """
    if not grid or not isinstance(grid, list):
        raise ValueError('Grid must be a non-empty list of strings.')

    rows = len(grid)
    cols = len(grid[0])
    if cols == 0:
        raise ValueError('Grid rows cannot be empty.')

    start_pos: Optional[Coordinate] = None
    goal_pos: Optional[Coordinate] = None

    for r, row_str in enumerate(grid):
        if len(row_str) != cols:
            raise ValueError(f'Inconsistent column count at row {r}: expected {cols}, found {len(row_str)}.')
        for c, char in enumerate(row_str):
            if char not in VALID_CELL_CHARS:
                raise ValueError(f"Invalid character '{char}' at coordinate ({r}, {c}).")
            if char == 'S':
                if start_pos is not None:
                    raise ValueError(f'Duplicate start position S detected at ({r}, {c}).')
                start_pos = (r, c)
            elif char == 'G':
                if goal_pos is not None:
                    raise ValueError(f'Duplicate goal position G detected at ({r}, {c}).')
                goal_pos = (r, c)

    if start_pos is None:
        raise ValueError("Missing start position 'S' in grid.")
    if goal_pos is None:
        raise ValueError("Missing goal position 'G' in grid.")

    return GridInfo(grid=grid, rows=rows, cols=cols, start=start_pos, goal=goal_pos)


def get_valid_neighbors(pos: Coordinate, grid_info: GridInfo) -> List[Tuple[Coordinate, int]]:
    """Generates valid 4-connected neighboring coordinates and entering costs."""
    r, c = pos
    neighbors: List[Tuple[Coordinate, int]] = []
    for dr, dc in DIRECTIONS:
        nr, nc = r + dr, c + dc
        if 0 <= nr < grid_info.rows and 0 <= nc < grid_info.cols:
            char = grid_info.grid[nr][nc]
            if char != WALL_CHAR:
                cost = COST_TABLE[char]
                neighbors.append(((nr, nc), cost))
    return neighbors


def reconstruct_path(came_from: Dict[Coordinate, Optional[Coordinate]], goal: Coordinate) -> List[Coordinate]:
    """Reconstructs the optimal trajectory from Start to Goal via backward pointers."""
    path: List[Coordinate] = []
    curr: Optional[Coordinate] = goal
    while curr is not None:
        path.append(curr)
        curr = came_from.get(curr)
    path.reverse()
    return path


def validate_path_solution(grid: List[str], result: SearchResult) -> bool:
    """Rigorous assertion checker ensuring path legality and cost consistency."""
    grid_info = parse_and_validate_grid(grid)
    if not result.found:
        assert len(result.path) == 0, 'Unsuccessful search must report empty path.'
        assert result.cost == float('inf'), 'Unsuccessful search must report infinite cost.'
        return True

    path = result.path
    assert len(path) >= 1, 'Successful path cannot be empty.'
    assert path[0] == grid_info.start, f'Path starts at {path[0]}, expected {grid_info.start}.'
    assert path[-1] == grid_info.goal, f'Path ends at {path[-1]}, expected {grid_info.goal}.'
    assert result.length == len(path) - 1, f'Path length ({result.length}) != moves ({len(path)-1}).'

    recalculated_cost = 0
    for i in range(1, len(path)):
        prev, curr = path[i - 1], path[i]
        step_dist = abs(curr[0] - prev[0]) + abs(curr[1] - prev[1])
        assert step_dist == 1, f'Non-cardinal step from {prev} to {curr}.'
        r, c = curr
        char = grid_info.grid[r][c]
        assert char != WALL_CHAR, f'Path traverses obstacle at ({r}, {c}).'
        recalculated_cost += COST_TABLE[char]

    assert recalculated_cost == result.cost, (
        f'Cost mismatch: recomputed {recalculated_cost} != reported {result.cost}.'
    )
    return True


def render_ascii_path(grid: List[str], path: List[Coordinate]) -> str:
    """Renders grid with '*' along the trajectory while preserving 'S' and 'G'."""
    grid_info = parse_and_validate_grid(grid)
    path_set = set(path)
    rendered = []
    for r in range(grid_info.rows):
        row_chars = []
        for c in range(grid_info.cols):
            char = grid_info.grid[r][c]
            if char in ('S', 'G'):
                row_chars.append(char)
            elif (r, c) in path_set:
                row_chars.append('*')
            else:
                row_chars.append(char)
        rendered.append(''.join(row_chars))
    return '\n'.join(rendered)


# ==============================================================================
# UNIFORM COST SEARCH (UCS)
# ==============================================================================

def uniform_cost_search(grid: List[str], map_name: str = 'Grid') -> SearchResult:
    """
    Executes Uniform Cost Search (UCS) from 'S' to 'G'.
    Prioritizes nodes by accumulated path cost g(n).
    """
    grid_info = parse_and_validate_grid(grid)
    start_time = time.perf_counter()

    # Priority queue storing: (g_cost, sequence_id, current_coordinate)
    frontier: List[Tuple[int, int, Coordinate]] = []
    counter = 0
    heapq.heappush(frontier, (0, counter, grid_info.start))

    came_from: Dict[Coordinate, Optional[Coordinate]] = {grid_info.start: None}
    cost_so_far: Dict[Coordinate, int] = {grid_info.start: 0}

    nodes_expanded = 0
    max_frontier_size = len(frontier)

    while frontier:
        max_frontier_size = max(max_frontier_size, len(frontier))
        current_g, _, current_node = heapq.heappop(frontier)

        # Skip stale entries where a cheaper path to current_node was already found
        if current_g > cost_so_far[current_node]:
            continue

        nodes_expanded += 1

        # Goal check upon removal from priority queue (ensures cost optimality)
        if current_node == grid_info.goal:
            elapsed_ms = (time.perf_counter() - start_time) * 1000.0
            path = reconstruct_path(came_from, grid_info.goal)
            return SearchResult(
                algorithm='Uniform Cost Search (UCS)',
                map_name=map_name,
                found=True,
                path=path,
                length=len(path) - 1,
                cost=cost_so_far[grid_info.goal],
                nodes_expanded=nodes_expanded,
                max_frontier_size=max_frontier_size,
                execution_time_ms=elapsed_ms,
            )

        for neighbor, move_cost in get_valid_neighbors(current_node, grid_info):
            new_cost = current_g + move_cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current_node
                counter += 1
                heapq.heappush(frontier, (new_cost, counter, neighbor))
                max_frontier_size = max(max_frontier_size, len(frontier))

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    return SearchResult(
        algorithm='Uniform Cost Search (UCS)',
        map_name=map_name,
        found=False,
        path=[],
        length=0,
        cost=float('inf'),
        nodes_expanded=nodes_expanded,
        max_frontier_size=max_frontier_size,
        execution_time_ms=elapsed_ms,
    )


# ==============================================================================
# A* SEARCH WITH MANHATTAN DISTANCE HEURISTIC
# ==============================================================================

def manhattan_distance(p1: Coordinate, p2: Coordinate) -> int:
    """Calculates L1 Manhattan distance: |r1 - r2| + |c1 - c2|."""
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def a_star_search(grid: List[str], map_name: str = 'Grid') -> SearchResult:
    """
    Executes A* Search from 'S' to 'G' utilizing Manhattan distance heuristic.
    Prioritizes frontier nodes by f(n) = g(n) + h(n).
    """
    grid_info = parse_and_validate_grid(grid)
    start_time = time.perf_counter()

    # Priority queue storing: (f_cost, sequence_id, current_coordinate, g_cost)
    frontier: List[Tuple[int, int, Coordinate, int]] = []
    counter = 0
    h_start = manhattan_distance(grid_info.start, grid_info.goal)
    heapq.heappush(frontier, (h_start, counter, grid_info.start, 0))

    came_from: Dict[Coordinate, Optional[Coordinate]] = {grid_info.start: None}
    cost_so_far: Dict[Coordinate, int] = {grid_info.start: 0}

    nodes_expanded = 0
    max_frontier_size = len(frontier)

    while frontier:
        max_frontier_size = max(max_frontier_size, len(frontier))
        current_f, _, current_node, current_g = heapq.heappop(frontier)

        # Skip stale entries
        if current_g > cost_so_far[current_node]:
            continue

        nodes_expanded += 1

        # Goal check upon removal from priority queue (ensures optimality with admissible h)
        if current_node == grid_info.goal:
            elapsed_ms = (time.perf_counter() - start_time) * 1000.0
            path = reconstruct_path(came_from, grid_info.goal)
            return SearchResult(
                algorithm='A* Search',
                map_name=map_name,
                found=True,
                path=path,
                length=len(path) - 1,
                cost=cost_so_far[grid_info.goal],
                nodes_expanded=nodes_expanded,
                max_frontier_size=max_frontier_size,
                execution_time_ms=elapsed_ms,
            )

        for neighbor, move_cost in get_valid_neighbors(current_node, grid_info):
            new_g = current_g + move_cost
            if neighbor not in cost_so_far or new_g < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_g
                came_from[neighbor] = current_node
                counter += 1
                h_cost = manhattan_distance(neighbor, grid_info.goal)
                f_cost = new_g + h_cost
                heapq.heappush(frontier, (f_cost, counter, neighbor, new_g))
                max_frontier_size = max(max_frontier_size, len(frontier))

    elapsed_ms = (time.perf_counter() - start_time) * 1000.0
    return SearchResult(
        algorithm='A* Search',
        map_name=map_name,
        found=False,
        path=[],
        length=0,
        cost=float('inf'),
        nodes_expanded=nodes_expanded,
        max_frontier_size=max_frontier_size,
        execution_time_ms=elapsed_ms,
    )


# ==============================================================================
# MAIN EXECUTION & BENCHMARK SUITE
# ==============================================================================

def print_result_block(res: SearchResult, grid: List[str]) -> None:
    cost_str = f'{res.cost:.0f}' if res.found else 'N/A'
    len_str = f'{res.length}' if res.found else 'N/A'
    print(f"\n{'='*60}")
    print(f"=== {res.map_name}: {res.algorithm} Results ===")
    print(f"{'='*60}")
    print(f"Status:             {res.status}")
    if not res.found:
        print("No path found")
    print(f"Path Coordinates:   {res.path}")
    print(f"Path Length (moves):{len_str}")
    print(f"Path Cost:          {cost_str}")
    print(f"Nodes Expanded:     {res.nodes_expanded}")
    print(f"Max Frontier Size:  {res.max_frontier_size}")
    print(f"Runtime (ms):       {res.execution_time_ms:.3f} ms")
    if res.found:
        print("\nASCII Path Representation:")
        print(render_ascii_path(grid, res.path))


def main() -> None:
    print("Warehouse Delivery Robot: Discrete Pathfinding Search Algorithms")
    print("=" * 70)

    maps = [('MAP_A', MAP_A), ('MAP_B', MAP_B), ('MAP_C', MAP_C)]
    all_results = []

    # Run benchmarks on MAP_A, MAP_B, MAP_C
    for name, grid in maps:
        res_ucs = uniform_cost_search(grid, map_name=name)
        validate_path_solution(grid, res_ucs)
        print_result_block(res_ucs, grid)
        all_results.append(res_ucs)

        res_astar = a_star_search(grid, map_name=name)
        validate_path_solution(grid, res_astar)
        print_result_block(res_astar, grid)
        all_results.append(res_astar)

    # Active GRID Demonstration
    grid_name = 'MAP_A' if GRID == MAP_A else 'MAP_B' if GRID == MAP_B else 'MAP_C' if GRID == MAP_C else 'Custom GRID'
    print(f"\n{'#'*70}")
    print(f"### ACTIVE GRID DEMONSTRATION (Configured: {grid_name})")
    print(f"{'#'*70}")
    demo_ucs = uniform_cost_search(GRID, map_name=f"{grid_name} (Demo UCS)")
    demo_astar = a_star_search(GRID, map_name=f"{grid_name} (Demo A*)")
    print(f"UCS -> Status: {demo_ucs.status} | Cost: {demo_ucs.cost if demo_ucs.found else 'N/A'} | Moves: {demo_ucs.length if demo_ucs.found else 'N/A'} | Expanded: {demo_ucs.nodes_expanded}")
    print(f"A*  -> Status: {demo_astar.status} | Cost: {demo_astar.cost if demo_astar.found else 'N/A'} | Moves: {demo_astar.length if demo_astar.found else 'N/A'} | Expanded: {demo_astar.nodes_expanded}")

    # Summary Table
    print(f"\n{'='*85}")
    print(f"{'Map':<8} {'Algorithm':<28} {'Status':<15} {'Moves':<8} {'Cost':<8} {'Expanded':<10} {'Frontier':<10}")
    print(f"{'='*85}")
    for res in all_results:
        cost_str = f"{res.cost:.0f}" if res.found else "N/A"
        len_str = str(res.length) if res.found else "N/A"
        print(f"{res.map_name:<8} {res.algorithm:<28} {res.status:<15} {len_str:<8} {cost_str:<8} {res.nodes_expanded:<10} {res.max_frontier_size:<10}")
    print(f"{'='*85}\n")


if __name__ == '__main__':
    main()
