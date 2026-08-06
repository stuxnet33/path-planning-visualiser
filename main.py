import matplotlib.pyplot as plt
import matplotlib
from grid import Grid, draw_grid, pixel_to_cell, ROWS, COLS
from astar import astar
from rrt import rrt
from grid import Grid, draw_grid, pixel_to_cell, ROWS, COLS, save_grid, load_grid

# Use TkAgg backend for interactive window
matplotlib.use('TkAgg')

# ── State ──────────────────────────────────────────────────────────────────────
grid = Grid(ROWS, COLS)
mode = 'wall'        # current interaction mode: wall, start, end
algorithm = 'astar'  # which algorithm to run: astar, rrt
running = False      # is a search currently running?
mouse_held = False   # is the mouse button held down?

# ── Setup figure ───────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 9))
fig.patch.set_facecolor('#1a1a2e')
plt.subplots_adjust(bottom=0.18)
draw_grid(ax, grid)

# ── Instructions text ──────────────────────────────────────────────────────────
instructions = (
    "LEFT CLICK: draw wall   |   RIGHT CLICK: erase wall\n"
    "S: set start   |   E: set end   |   SPACE: run A*\n"
    "T: switch to RRT   |   C: clear search   |   R: full reset\n"
    "W: save maze   |   L: load maze"
)
fig.text(0.5, 0.02, instructions, ha='center', va='bottom',
         fontsize=8.5, color='white',
         fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='#2d2d44', alpha=0.8))

# Status bar
status_text = fig.text(0.5, 0.13, 'Ready — draw walls then press SPACE to run A*',
                        ha='center', va='bottom', fontsize=9,
                        color='#00ff88', fontfamily='monospace')


def update_status(msg, colour='#00ff88'):
    status_text.set_text(msg)
    status_text.set_color(colour)
    fig.canvas.draw_idle()


def refresh():
    """Redraws the grid. Passed as draw_callback to algorithms for animation."""
    draw_grid(ax, grid)
    fig.canvas.draw_idle()
    fig.canvas.flush_events()


# ── Mouse events ───────────────────────────────────────────────────────────────
def on_press(event):
    global mouse_held, mode
    if running or event.inaxes != ax:
        return
    mouse_held = True
    handle_click(event)


def on_release(event):
    global mouse_held
    mouse_held = False


def on_motion(event):
    """Handles click-and-drag for drawing walls."""
    if not mouse_held or running or event.inaxes != ax:
        return
    if event.button == 1:  # left — draw wall
        row, col = pixel_to_cell(event.xdata, event.ydata, grid)
        grid.set_wall(row, col)
        refresh()
    elif event.button == 3:  # right — erase wall
        row, col = pixel_to_cell(event.xdata, event.ydata, grid)
        grid.clear_wall(row, col)
        refresh()


def handle_click(event):
    global mode
    if event.xdata is None or event.ydata is None:
        return
    row, col = pixel_to_cell(event.xdata, event.ydata, grid)

    if event.button == 1:  # left click
        if mode == 'start':
            grid.set_start(row, col)
            update_status(f'Start set at ({row}, {col})')
            mode = 'wall'
        elif mode == 'end':
            grid.set_end(row, col)
            update_status(f'End set at ({row}, {col})')
            mode = 'wall'
        else:
            grid.set_wall(row, col)
        refresh()

    elif event.button == 3:  # right click — erase
        grid.clear_wall(row, col)
        refresh()


# ── Keyboard events ─────────────────────────────────────────────────────────────
def on_key(event):
    global mode, algorithm, running

    if event.key == 's':
        mode = 'start'
        update_status("Click anywhere to set START (green)", '#00ff88')

    elif event.key == 'e':
        mode = 'end'
        update_status("Click anywhere to set END (red)", '#ff4444')

    elif event.key == 'c':
        grid.reset_search()
        refresh()
        update_status("Search cleared — ready to run again")

    elif event.key == 'r':
        grid.full_reset()
        refresh()
        update_status("Full reset — grid cleared")

    elif event.key == 'w':
        save_grid(grid)
        update_status("Maze saved — press L after restart to reload", '#ffaa00')

    elif event.key == 'l':
        if load_grid(grid):
            refresh()
            update_status("Maze loaded", '#00ff88')
        else:
            update_status("No saved maze found", '#ff4444')

    elif event.key == 't':
        algorithm = 'rrt' if algorithm == 'astar' else 'astar'
        update_status(f"Algorithm switched to: {algorithm.upper()}", '#ffaa00')

    elif event.key == ' ':
        if running:
            return
        if not grid.start or not grid.end:
            update_status("Set START (S) and END (E) first", '#ff4444')
            return

        grid.reset_search()
        running = True

        if algorithm == 'astar':
            update_status("Running A*...", '#4488ff')
            fig.canvas.draw()
            path, explored = astar(grid, draw_callback=refresh)
            if path:
                update_status(
                    f"A* done — path length: {len(path)} cells, explored: {explored} cells",
                    '#00ff88'
                )
            else:
                update_status("A* — no path found", '#ff4444')

        elif algorithm == 'rrt':
            update_status("Running RRT...", '#aa44ff')
            fig.canvas.draw()

            path = []
            iterations = 0
            attempts = 0
            max_attempts = 3

            while not path and attempts < max_attempts:
                attempts += 1
                if attempts > 1:
                    grid.reset_search()
                    update_status(f"RRT retry {attempts}/{max_attempts}...", '#aa44ff')
                    fig.canvas.draw()

                path, iterations = rrt(grid, draw_callback=refresh)

            if path:
                update_status(
                    f"RRT done — path: {len(path)} cells, iterations: {iterations}, attempts: {attempts}",
                    '#00ff88'
                )
            else:
                update_status(f"RRT failed after {max_attempts} attempts", '#ff4444')

        running = False
        refresh()


# ── Connect events ──────────────────────────────────────────────────────────────
fig.canvas.mpl_connect('button_press_event', on_press)
fig.canvas.mpl_connect('button_release_event', on_release)
fig.canvas.mpl_connect('motion_notify_event', on_motion)
fig.canvas.mpl_connect('key_press_event', on_key)

plt.title("Path Planning Visualiser — A* and RRT",
          color='white', fontsize=11, pad=10)
ax.set_facecolor('#1a1a2e')

update_status('Ready — draw walls, press S for start, E for end, SPACE to run')
plt.show()