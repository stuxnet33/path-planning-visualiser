import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# Grid dimensions
ROWS = 40
COLS = 40
CELL_SIZE = 1

# Colours (RGB 0-1 scale for matplotlib)
WHITE  = (1.0, 1.0, 1.0)   # unvisited
BLACK  = (0.0, 0.0, 0.0)   # wall
GREEN  = (0.0, 0.8, 0.2)   # start
RED    = (0.9, 0.1, 0.1)   # end
BLUE   = (0.4, 0.6, 1.0)   # explored by A*
YELLOW = (1.0, 0.85, 0.0)  # final path
PURPLE = (0.6, 0.2, 0.9)   # RRT tree
ORANGE = (1.0, 0.5, 0.0)   # RRT path


class Grid:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols

        # 2D array storing the state of each cell
        # 0 = free, 1 = wall
        self.walls = np.zeros((rows, cols), dtype=int)

        # Colour array for rendering — each cell has an RGB colour
        self.colours = np.ones((rows, cols, 3))  # start all white

        self.start = None  # (row, col)
        self.end   = None  # (row, col)

    def set_wall(self, row, col):
        """Mark a cell as a wall."""
        if (row, col) == self.start or (row, col) == self.end:
            return  # don't overwrite start or end
        self.walls[row][col] = 1
        self.colours[row][col] = BLACK

    def clear_wall(self, row, col):
        """Remove a wall from a cell."""
        self.walls[row][col] = 0
        self.colours[row][col] = WHITE

    def set_start(self, row, col):
        """Set the start cell."""
        if self.start:
            # Clear the old start
            self.colours[self.start[0]][self.start[1]] = WHITE
        self.start = (row, col)
        self.walls[row][col] = 0  # start can't be a wall
        self.colours[row][col] = GREEN

    def set_end(self, row, col):
        """Set the end cell."""
        if self.end:
            # Clear the old end
            self.colours[self.end[0]][self.end[1]] = WHITE
        self.end = (row, col)
        self.walls[row][col] = 0  # end can't be a wall
        self.colours[row][col] = RED

    def colour_cell(self, row, col, colour):
        """
        Colour a single cell. Used by the algorithms to show
        their progress — explored cells, path cells, tree branches.
        Doesn't overwrite start or end.
        """
        if (row, col) == self.start or (row, col) == self.end:
            return
        self.colours[row][col] = colour

    def is_wall(self, row, col):
        """Returns True if this cell is a wall."""
        return self.walls[row][col] == 1

    def in_bounds(self, row, col):
        """Returns True if (row, col) is inside the grid."""
        return 0 <= row < self.rows and 0 <= col < self.cols

    def get_neighbours(self, row, col):
        """
        Returns all valid non-wall neighbours of a cell.
        We allow 8-directional movement (including diagonals).
        """
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),   # up, down, left, right
            (-1,-1), (-1,1), (1,-1),  (1, 1)      # diagonals
        ]
        neighbours = []
        for dr, dc in directions:
            nr, nc = row + dr, col + dc
            if self.in_bounds(nr, nc) and not self.is_wall(nr, nc):
                neighbours.append((nr, nc))
        return neighbours

    def reset_search(self):
        """
        Clears all algorithm colouring (explored, path, tree)
        but keeps walls, start and end in place.
        Ready to run a new search.
        """
        for r in range(self.rows):
            for c in range(self.cols):
                if self.walls[r][c] == 1:
                    self.colours[r][c] = BLACK
                elif (r, c) == self.start:
                    self.colours[r][c] = GREEN
                elif (r, c) == self.end:
                    self.colours[r][c] = RED
                else:
                    self.colours[r][c] = WHITE

    def full_reset(self):
        """Clears everything including walls, start and end."""
        self.walls = np.zeros((self.rows, self.cols), dtype=int)
        self.colours = np.ones((self.rows, self.cols, 3))
        self.start = None
        self.end = None


def draw_grid(ax, grid):
    """
    Renders the grid onto a matplotlib axes object.
    Called every frame to update the display.
    """
    ax.clear()
    ax.imshow(
        grid.colours,
        origin='upper',
        extent=[0, grid.cols, 0, grid.rows],
        aspect='equal'
    )

    # Draw grid lines
    for r in range(grid.rows + 1):
        ax.axhline(r, color='#cccccc', linewidth=0.3)
    for c in range(grid.cols + 1):
        ax.axvline(c, color='#cccccc', linewidth=0.3)

    ax.set_xlim(0, grid.cols)
    ax.set_ylim(0, grid.rows)
    ax.set_xticks([])
    ax.set_yticks([])


def pixel_to_cell(x, y, grid):
    """
    Converts a matplotlib click coordinate (x, y)
    to a grid cell (row, col).
    """
    col = int(x)
    row = int(grid.rows - y)  # flip because imshow origin is upper
    row = max(0, min(grid.rows - 1, row))
    col = max(0, min(grid.cols - 1, col))
    return row, col

import json

def save_grid(grid, filename="saved_maze.json"):
    """
    Saves the current grid state to a JSON file.
    Stores walls, start and end position.
    """
    data = {
        "walls": grid.walls.tolist(),
        "start": grid.start,
        "end": grid.end,
        "rows": grid.rows,
        "cols": grid.cols,
    }
    with open(filename, 'w') as f:
        json.dump(data, f)
    print(f"Maze saved to {filename}")


def load_grid(grid, filename="saved_maze.json"):
    """
    Loads a previously saved grid state.
    Restores walls, start and end position.
    """
    try:
        with open(filename, 'r') as f:
            data = json.load(f)

        grid.walls = np.array(data["walls"])
        grid.start = tuple(data["start"]) if data["start"] else None
        grid.end   = tuple(data["end"])   if data["end"]   else None

        # Rebuild colours from walls/start/end
        grid.reset_search()
        if grid.start:
            grid.colours[grid.start[0]][grid.start[1]] = GREEN
        if grid.end:
            grid.colours[grid.end[0]][grid.end[1]] = RED

        print(f"Maze loaded from {filename}")
        return True

    except FileNotFoundError:
        print(f"No saved maze found at {filename}")
        return False