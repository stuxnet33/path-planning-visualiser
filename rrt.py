import random
import math
from grid import PURPLE, ORANGE


def distance(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def bresenham(r0, c0, r1, c1):
    """
    Bresenham's line algorithm.
    Returns every integer cell the line passes through.
    This is the correct way to walk a line on a discrete grid.
    """
    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0

    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc

    return cells


def is_collision_free(grid, from_node, to_node):
    """
    Uses Bresenham to check every cell the line passes through.
    Returns False if any cell is a wall or out of bounds.
    """
    cells = bresenham(from_node[0], from_node[1], to_node[0], to_node[1])
    for r, c in cells:
        if not grid.in_bounds(r, c) or grid.is_wall(r, c):
            return False
    return True


def nearest_node(tree_list, point):
    """Finds the nearest node in the tree to the given point."""
    return min(tree_list, key=lambda node: distance(node, point))


def steer(from_node, to_point, step_size):
    """
    Moves step_size toward to_point.
    Always returns exact integer coordinates.
    """
    dr = to_point[0] - from_node[0]
    dc = to_point[1] - from_node[1]
    d = math.sqrt(dr**2 + dc**2)

    if d == 0:
        return from_node

    new_row = int(round(from_node[0] + (dr / d) * step_size))
    new_col = int(round(from_node[1] + (dc / d) * step_size))
    return (new_row, new_col)


def reconstruct_path(parent, end):
    """
    Iterative path reconstruction.
    Traces parent links from end back to start.
    """
    path = []
    current = end
    visited = set()

    while current is not None:
        if current in visited:
            return []  # cycle detected — invalid path
        visited.add(current)
        path.append(current)
        current = parent.get(current)

    path.reverse()
    return path


def rrt(grid, max_iterations=15000, step_size=1, goal_threshold=1, draw_callback=None):
    """
    RRT — computation runs completely first, animation happens after.
    This avoids matplotlib freezing mid-algorithm.
    """
    start = grid.start
    end = grid.end

    if not start or not end:
        print("Set start and end first")
        return [], 0

    tree_list = [start]
    tree_set = {start}
    parent = {start: None}

    # Track all nodes added for animation afterwards
    nodes_added = []

    solution_path = []
    solution_iteration = 0

    for iteration in range(max_iterations):

        # Goal bias 15%
        if random.random() < 0.15:
            rand_point = end
        else:
            rand_point = (
                random.randint(0, grid.rows - 1),
                random.randint(0, grid.cols - 1)
            )

        nearest = nearest_node(tree_list, rand_point)
        new_node = steer(nearest, rand_point, step_size)

        # Validate
        if not grid.in_bounds(new_node[0], new_node[1]):
            continue
        if grid.is_wall(new_node[0], new_node[1]):
            continue
        if new_node in tree_set:
            continue
        if not is_collision_free(grid, nearest, new_node):
            continue

        # Add to tree
        tree_list.append(new_node)
        tree_set.add(new_node)
        parent[new_node] = nearest
        nodes_added.append(new_node)

        # Check if goal reached
        if distance(new_node, end) <= goal_threshold:
            if is_collision_free(grid, new_node, end):
                parent[end] = new_node

                path = reconstruct_path(parent, end)

                # Validate — must be at least start + 1 node + end
                if len(path) >= 2 and path[0] == start:
                    solution_path = path
                    solution_iteration = iteration
                    break
                else:
                    # Bad path — remove end from parent and keep searching
                    del parent[end]
                    continue

    # ── Animation phase ─────────────────────────────────────────
    # Now that computation is done, colour everything at once
    # This runs independently of the algorithm so matplotlib can't interfere

    # Colour tree nodes
    for i, node in enumerate(nodes_added):
        grid.colour_cell(node[0], node[1], PURPLE)
        # Only redraw occasionally to keep it fast
        if draw_callback and i % 50 == 0:
            draw_callback()

    # Colour path
    if solution_path:
        for cell in solution_path[1:-1]:
            grid.colour_cell(cell[0], cell[1], ORANGE)
        if draw_callback:
            draw_callback()

        print(f"RRT solved: {len(solution_path)} cells, {solution_iteration} iterations")
        return solution_path, solution_iteration
    else:
        if draw_callback:
            draw_callback()
        print(f"RRT failed after {max_iterations} iterations")
        return [], max_iterations