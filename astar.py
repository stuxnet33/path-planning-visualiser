import heapq
import math
from grid import BLUE, YELLOW, GREEN, RED


def heuristic(a, b):
    """
    Euclidean distance between two cells.
    This is our estimate of how far cell a is from cell b.
    It's admissible — never overestimates — so A* finds the optimal path.
    We use Euclidean rather than Manhattan because we allow diagonal movement.
    """
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def movement_cost(current, neighbour):
    """
    The actual cost to move from current to neighbour.
    Diagonal moves cost sqrt(2) ~ 1.414, straight moves cost 1.
    This reflects real distance rather than treating all moves equally.
    """
    dr = abs(current[0] - neighbour[0])
    dc = abs(current[1] - neighbour[1])
    if dr == 1 and dc == 1:
        return math.sqrt(2)  # diagonal
    return 1.0  # straight


def reconstruct_path(came_from, current):
    """
    Traces back from the goal to the start using the came_from dictionary.
    came_from[node] = the node we came from to reach node.
    We follow this chain backwards until we reach the start (which has no entry).
    Returns the path as a list from start to goal.
    """
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def astar(grid, draw_callback=None):
    """
    A* search algorithm.

    Parameters:
        grid: the Grid object containing walls, start, end
        draw_callback: a function called after each step to update the display.
                       This is what creates the live animation effect.

    Returns:
        path: list of (row, col) tuples from start to goal, or empty list if no path.
        explored_count: how many cells were explored (useful for comparison with RRT)

    The algorithm:
    1. Start at the start cell with f = 0 + h(start, end)
    2. Always expand the cell with the lowest f score
    3. For each neighbour, calculate g (cost so far) and h (heuristic to goal)
    4. If we find a better path to a neighbour, update it
    5. Stop when we reach the goal or exhaust all possibilities
    """

    start = grid.start
    end   = grid.end

    if not start or not end:
        print("Set start and end points first")
        return [], 0

    # g_score[node] = actual cost from start to node
    # Initialise all to infinity — we haven't found a path to anywhere yet
    g_score = {}
    g_score[start] = 0

    # came_from[node] = which node we came from to reach node
    # Used to reconstruct the path at the end
    came_from = {}

    # The open set is a priority queue — we always explore the lowest f-score node next
    # Each entry is (f_score, count, node) — count breaks ties when f scores are equal
    # We use a counter because heapq needs all elements to be comparable,
    # and tuples are compared element by element — if f scores tie, it would
    # try to compare the nodes (tuples) which works but isn't meaningful.
    # The counter ensures we never need to compare nodes directly.
    count = 0
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, end), count, start))

    # Track which nodes are in the open set (heapq doesn't support fast lookup)
    open_set_hash = {start}

    explored_count = 0

    while open_set:
        # Pop the node with the lowest f score
        _, _, current = heapq.heappop(open_set)
        open_set_hash.discard(current)

        # Goal reached — reconstruct and colour the path
        if current == end:
            path = reconstruct_path(came_from, current)
            for cell in path:
                grid.colour_cell(cell[0], cell[1], YELLOW)
                if draw_callback:
                    draw_callback()
            return path, explored_count

        # Explore all neighbours
        for neighbour in grid.get_neighbours(current[0], current[1]):
            tentative_g = g_score.get(current, float('inf')) + movement_cost(current, neighbour)

            # If we found a better path to this neighbour, update it
            if tentative_g < g_score.get(neighbour, float('inf')):
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g
                f_score = tentative_g + heuristic(neighbour, end)

                if neighbour not in open_set_hash:
                    count += 1
                    heapq.heappush(open_set, (f_score, count, neighbour))
                    open_set_hash.add(neighbour)

                    # Colour this cell as explored (but not start/end)
                    grid.colour_cell(neighbour[0], neighbour[1], BLUE)
                    explored_count += 1

                    if draw_callback:
                        draw_callback()

    # Open set exhausted — no path exists
    print("No path found")
    return [], explored_count